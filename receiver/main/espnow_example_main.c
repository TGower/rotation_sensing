/* Receiver Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include "driver/i2c.h"
// #include "driver/usb_serial_jtag.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "espnow_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LEDC_IO 48
#define ESPNOW_MAXDELAY 512

static const char *TAG = "receiver";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF};
static uint8_t g_target_mac[ESP_NOW_ETH_ALEN] = {0};
static bool g_target_mac_set = false;
static SemaphoreHandle_t g_send_cb_sem;
static volatile bool g_send_status = false;

// I2C Configuration for TMAG5273
#define I2C_MASTER_SCL_IO 12        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 11        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define TMAG5273_ADDR 0x22 // Found Address (TMAG5273A2)

// Registers from Datasheet
#define TMAG_REG_DEVICE_CONFIG_1 0x00
#define TMAG_REG_DEVICE_CONFIG_2 0x01
#define TMAG_REG_SENSOR_CONFIG_1 0x02
#define TMAG_REG_MAN_ID_LSB 0x0E
#define TMAG_REG_MAN_ID_MSB 0x0F
#define TMAG_REG_RESULT_X 0x12 // X_MSB_RESULT

// Global Data Storage
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} tmag_data_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  int64_t local_timestamp;
  int64_t sender_timestamp;
  int16_t tmag_x;
  int16_t tmag_y;
  int16_t tmag_z;
  int8_t rssi;
  uint16_t csi_len;
  int64_t csi_timestamp;
  uint8_t csi_data[512];
} csi_log_packet_t;

typedef struct {
  tmag_data_t tmag;
  int64_t sender_timestamp;
} shared_state_t;

static shared_state_t g_shared_state = {0};
static SemaphoreHandle_t g_data_mutex;

#define BURST_SIZE 4000
#define PACKET_PAYLOAD_SIZE 131
static uint8_t (*g_burst_buffer)[PACKET_PAYLOAD_SIZE]; // Pointer to array
static int g_burst_idx = 0;
static bool g_is_flushing = false;
static SemaphoreHandle_t g_flush_sem;

// Function to init I2C with specific pins
static esp_err_t i2c_master_init(int sda, int scl) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = scl,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
  if (err != ESP_OK)
    return err;
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Helper to write register
static esp_err_t tmag_write_byte(uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_write_to_device(I2C_MASTER_NUM, TMAG5273_ADDR, write_buf,
                                    sizeof(write_buf),
                                    1000 / portTICK_PERIOD_MS);
}

// Simple Read Register
static esp_err_t tmag_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
  return i2c_master_write_read_device(I2C_MASTER_NUM, TMAG5273_ADDR, &reg_addr,
                                      1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Task to read TMAG continuously
static void tmag_task(void *pvParameter) {
  uint8_t raw_data[6];

  // Configuration
  // 1. Enable X, Y, Z channels
  // Reg: SENSOR_CONFIG_1 (0x02) -> 0x70
  esp_err_t ret = tmag_write_byte(TMAG_REG_SENSOR_CONFIG_1, 0x70);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write SENSOR_CONFIG_1");
  }

  // 2. Set to Continuous Measure Mode
  // Reg: DEVICE_CONFIG_2 (0x01) -> 0x02
  ret = tmag_write_byte(TMAG_REG_DEVICE_CONFIG_2, 0x02);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write DEVICE_CONFIG_2");
  }

  ESP_LOGI(TAG, "Sensor configured for Continuous Mode + XYZ");

  // Verify Manufacturer ID
  uint8_t id_lsb = 0, id_msb = 0;
  tmag_read_bytes(TMAG_REG_MAN_ID_LSB, &id_lsb, 1);
  tmag_read_bytes(TMAG_REG_MAN_ID_MSB, &id_msb, 1);
  ESP_LOGI(TAG, "TMAG Manufacturer ID: 0x%02X%02X", id_msb, id_lsb);

  while (1) {
    // Read 6 bytes starting from X_MSB (0x12)
    esp_err_t ret = tmag_read_bytes(TMAG_REG_RESULT_X, raw_data, 6);

    if (ret == ESP_OK) {
      if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
        g_shared_state.tmag.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        g_shared_state.tmag.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        g_shared_state.tmag.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        xSemaphoreGive(g_data_mutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Fast poll
  }
}

// CSI Callback - Now the MAIN TRIGGER for logging
#define CSI_LOG_MAGIC 0xDEADBEEF

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf || !info->len) {
    return;
  }

  if (g_is_flushing) {
    return;
  }

  // 1. Prepare Packet
  static csi_log_packet_t packet;
  packet.magic = CSI_LOG_MAGIC;
  packet.local_timestamp = esp_timer_get_time();

  // 2. Grab Shared State (TMAG + Sender TS)
  if (xSemaphoreTakeFromISR(g_data_mutex, NULL) == pdTRUE) {
    packet.sender_timestamp = g_shared_state.sender_timestamp;
    packet.tmag_x = g_shared_state.tmag.x;
    packet.tmag_y = g_shared_state.tmag.y;
    packet.tmag_z = g_shared_state.tmag.z;
    xSemaphoreGiveFromISR(g_data_mutex, NULL);
  } else {
    // Should generally succeed, but if not we proceed with partial data?
    // Or drop? Let's drop to ensure integrity.
    return;
  }

  packet.rssi = info->rx_ctrl.rssi;
  packet.csi_timestamp = packet.local_timestamp; // Use same TS for now

  // Compact CSI Data: Keep only subcarriers 1-28 and 45-63 (Total 47)
  int out_idx = 0;
  // Ensure we don't read out of bounds if info->len is weird
  // CSI data is int8_t pairs (I,Q). info->len is bytes.
  // We expect 128 bytes (64 SCs) usually for 20MHz HT.

  const uint8_t *csi_raw = (const uint8_t *)info->buf;

  if (info->len >= 128) {
    for (int i = 1; i <= 28; i++) {
      packet.csi_data[out_idx++] = csi_raw[i * 2];
      packet.csi_data[out_idx++] = csi_raw[i * 2 + 1];
    }
    for (int i = 45; i <= 63; i++) {
      packet.csi_data[out_idx++] = csi_raw[i * 2];
      packet.csi_data[out_idx++] = csi_raw[i * 2 + 1];
    }
    packet.csi_len = out_idx; // 94 bytes
  } else {
    // Unexpected len, skip
    return;
  }

  // Header(37) + CSI(94) = 131 bytes
  // size_t write_len = 37 + packet.csi_len;

  // 3. Buffer Packet
  if (g_burst_idx < BURST_SIZE) {
    memcpy(g_burst_buffer[g_burst_idx], &packet, PACKET_PAYLOAD_SIZE);
    g_burst_idx++;

    if (g_burst_idx == BURST_SIZE) {
      // ESP_LOGI(TAG, "Burst full, starting flush");
      g_is_flushing = true;
      xSemaphoreGiveFromISR(g_flush_sem, NULL);
    }
  }
}

// Send Callback
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  g_send_status = (status == ESP_NOW_SEND_SUCCESS);
  xSemaphoreGiveFromISR(g_send_cb_sem, NULL);
}

// Task to flush the buffer
static void flush_task(void *pvParameter) {
  while (1) {
    if (xSemaphoreTake(g_flush_sem, portMAX_DELAY) == pdTRUE) {
      if (!g_target_mac_set) {
        ESP_LOGW(TAG, "Cannot flush: Target MAC not set yet!");
        // We have to drop this burst or just wait. Dropping to avoid deadlock
        // logic for now, but ideally we should wait. However, if we wait here,
        // we block. Let's just reset index and continue to allow new fresh
        // data? Or maybe just busy wait a bit? Let's drop and log error, user
        // needs to ensure collector is running.
        g_burst_idx = 0;
        g_is_flushing = false;
        continue;
      }

      ESP_LOGI(TAG, "Starting burst flush of %d packets to " MACSTR "...",
               BURST_SIZE, MAC2STR(g_target_mac));

      for (int i = 0; i < BURST_SIZE; i++) {
        esp_err_t err;
        int retries = 0;
        const int MAX_RETRIES = 50; // 50 retries * delay
        bool sent_successfully = false;

        while (!sent_successfully && retries < MAX_RETRIES) {
          // Reset status
          g_send_status = false;
          // Clear sem just in case
          xSemaphoreTake(g_send_cb_sem, 0);

          err = esp_now_send(g_target_mac, g_burst_buffer[i],
                             PACKET_PAYLOAD_SIZE);

          if (err == ESP_OK) {
            // Wait for ACK
            if (xSemaphoreTake(g_send_cb_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
              if (g_send_status) {
                sent_successfully = true;
              } else {
                // Send callback indicated failure (no ACK)
                retries++;
                vTaskDelay(1);
              }
            } else {
              // TImeout waiting for callback
              ESP_LOGW(TAG, "Send callback timeout");
              retries++;
            }
          } else {
            // Immediate send error (e.g. buffers full)
            if (err == ESP_ERR_ESPNOW_NO_MEM) {
              vTaskDelay(1); // Wait for buffers to clear
            } else {
              // Other error
              ESP_LOGE(TAG, "esp_now_send error: %d", err);
            }
            retries++;
          }
        }

        if (!sent_successfully) {
          ESP_LOGE(TAG,
                   "Failed to send packet %d after %d retries. Continuing...",
                   i, retries);
        }

        // Optional: reduce CPU usage
        // vTaskDelay(1); // Removed to go as fast as ACKs allow
      }
      ESP_LOGI(TAG, "Burst flush complete.");
      g_burst_idx = 0;
      g_is_flushing = false;
    }
  }
}

// ESP-NOW Receive Callback - PASSIVE, just updates timestamp
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {
  // Capture Target MAC
  if (!g_target_mac_set && recv_info->src_addr) {
    memcpy(g_target_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    g_target_mac_set = true;
    ESP_LOGI(TAG, "Discovered Target MAC: " MACSTR, MAC2STR(g_target_mac));

    // Add as peer if not exists
    if (!esp_now_is_peer_exist(g_target_mac)) {
      esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
      if (peer != NULL) {
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, g_target_mac, ESP_NOW_ETH_ALEN);
        esp_err_t add_err = esp_now_add_peer(peer);
        free(peer);
        if (add_err == ESP_OK) {
          ESP_LOGI(TAG, "Added Target as Peer");
        } else {
          ESP_LOGE(TAG, "Failed to add Target peer: %d", add_err);
        }
      }
    }
  }

  int64_t sender_timestamp = 0;
  if (len >= sizeof(int64_t)) {
    memcpy(&sender_timestamp, data, sizeof(int64_t));
  }

  if (xSemaphoreTake(g_data_mutex, 0) == pdTRUE) {
    g_shared_state.sender_timestamp = sender_timestamp;
    xSemaphoreGive(g_data_mutex);
  }
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(
      ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                          WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

  // Enable Promiscuous mode for CSI on some chips/versions
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  // CSI Config
  wifi_csi_config_t csi_config = {
      .lltf_en = true,
      .htltf_en = true,
      .stbc_htltf2_en = true,
      .ltf_merge_en = false,
      .channel_filter_en = false,
      .manu_scale = true,
      .shift = 2,
  };

  // Try to disable CSI first just in case
  esp_wifi_set_csi(false);

  esp_err_t res = esp_wifi_set_csi_config(&csi_config);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set CSI config: %d", res);
  } else {
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
  }
}

static esp_err_t example_espnow_init(void) {
  /* Initialize ESPNOW and register sending and receiving callback function. */
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

  /* Set primary master key. */
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

  /* Add broadcast peer information to peer list. */
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  if (peer == NULL) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    esp_now_deinit();
    return ESP_FAIL;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = CONFIG_ESPNOW_CHANNEL;
  peer->ifidx = ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(peer));
  free(peer);

  /* Set global ESPNOW rate to 24Mbps to handle high packet rate */
  esp_err_t err =
      esp_wifi_config_espnow_rate(ESPNOW_WIFI_IF, WIFI_PHY_RATE_24M);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Global rate config error: %d (%s)", err,
             esp_err_to_name(err));
  }

  return ESP_OK;
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting Receiver App...");
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize LED Strip and Dim it (User Request)
  // Note: Only if LED strip component is available and valid pin
  led_strip_handle_t led_strip;
  led_strip_config_t strip_config = {
      .strip_gpio_num = LEDC_IO,
      .max_leds = 1,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  led_strip_set_pixel(led_strip, 0, 8, 8, 8); // ~3% brightness (very dim)
  led_strip_refresh(led_strip);

  // Allocate large buffer in PSRAM
  size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "Free SPIRAM before alloc: %d bytes", free_spiram);

  g_burst_buffer =
      heap_caps_malloc(BURST_SIZE * PACKET_PAYLOAD_SIZE, MALLOC_CAP_SPIRAM);
  if (g_burst_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate burst buffer in SPIRAM! Requested: %d",
             BURST_SIZE * PACKET_PAYLOAD_SIZE);
    return;
  }
  ESP_LOGI(TAG, "Allocated %d bytes in SPIRAM for burst buffer",
           BURST_SIZE * PACKET_PAYLOAD_SIZE);

  g_data_mutex = xSemaphoreCreateMutex();
  g_flush_sem = xSemaphoreCreateBinary();
  g_send_cb_sem = xSemaphoreCreateBinary();

  // Init I2C Fixed
  ESP_LOGI(TAG, "Initializing I2C SDA=11 SCL=12");
  ESP_ERROR_CHECK(i2c_master_init(11, 12));

  // I2C Scan
  ESP_LOGI(TAG, "Scanning I2C...");
  for (int i = 0; i < 128; i++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Found I2C device at: 0x%02x", i);
    }
  }

  xTaskCreate(tmag_task, "tmag_task", 4096, NULL, 5, NULL);
  xTaskCreate(flush_task, "flush_task", 4096, NULL, 10, NULL);

  // Initialize USB Serial/JTAG driver for high-speed logging
  // usb_serial_jtag_driver_config_t cfg =
  // USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  // usb_serial_jtag_driver_install(&cfg);

  example_wifi_init();
  example_espnow_init();
}
