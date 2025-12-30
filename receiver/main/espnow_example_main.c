/* Receiver Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include "driver/i2c.h"
// #include "driver/usb_serial_jtag.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
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
#include "math.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LEDC_IO 48
#define ESPNOW_MAXDELAY 512

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz
#define DSHOT_ESC_GPIO_NUM_A 13
#define DSHOT_ESC_GPIO_NUM_B 14

#define RSSI_BUF_SIZE 6000
#define INTERPOLATION_INTERVAL_US 100
#define CORRELATION_WINDOW 1000

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

typedef struct {
  int8_t rssi;
  int64_t timestamp;
} rssi_sample_t;

typedef struct {
  rssi_sample_t buffer[RSSI_BUF_SIZE];
  int head;
  int tail;
  int64_t last_timestamp;
} rssi_circular_buffer_t;

typedef struct {
  uint16_t throttle;
  float vector_x;
  float vector_y;
} control_input_t;

typedef struct {
  float rotation_rate; // Radians per second? Or generic units? Let's say Hz for
                       // now or normalized.
  float phase_offset;
  int64_t last_peak_timestamp;
  float estimated_period_us;
} rotation_state_t;

static rssi_circular_buffer_t g_rssi_buf = {0};
static control_input_t g_control_input = {0};
static rotation_state_t g_rotation_state = {0};

static SemaphoreHandle_t g_data_mutex;

// DShot Handles
static rmt_channel_handle_t esc_chan_a = NULL;
static rmt_channel_handle_t esc_chan_b = NULL;
static rmt_encoder_handle_t dshot_encoder_a = NULL;
static rmt_encoder_handle_t dshot_encoder_b = NULL;
static led_strip_handle_t g_led_strip = NULL;

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

// Helper: Interpolate and Add to Circular Buffer
static void interpolate_rssi(int64_t timestamp, int8_t rssi) {
  // If buffer is empty, just add the first point
  if (g_rssi_buf.last_timestamp == 0) {
    g_rssi_buf.buffer[g_rssi_buf.head].rssi = rssi;
    g_rssi_buf.buffer[g_rssi_buf.head].timestamp = timestamp;
    g_rssi_buf.last_timestamp = timestamp;
    g_rssi_buf.head = (g_rssi_buf.head + 1) % RSSI_BUF_SIZE;
    // Tail stays 0 until full? Or just 0.
    return;
  }

  int64_t target_ts = g_rssi_buf.last_timestamp + INTERPOLATION_INTERVAL_US;

  // Safety: If gap is too large (> 100ms), reset
  if (timestamp - g_rssi_buf.last_timestamp > 100000) {
    g_rssi_buf.last_timestamp = timestamp;
    g_rssi_buf.buffer[g_rssi_buf.head].rssi = rssi;
    g_rssi_buf.buffer[g_rssi_buf.head].timestamp = timestamp;
    g_rssi_buf.head = (g_rssi_buf.head + 1) % RSSI_BUF_SIZE;
    if (g_rssi_buf.head == g_rssi_buf.tail) {
      g_rssi_buf.tail = (g_rssi_buf.tail + 1) % RSSI_BUF_SIZE;
    }
    return;
  }

  // Nearest Neighbor Interpolation for all uniform points between last_ts and
  // current ts
  while (target_ts <= timestamp) {
    // Nearest neighbor
    int8_t val;

    val = rssi;

    g_rssi_buf.buffer[g_rssi_buf.head].rssi = val;
    g_rssi_buf.buffer[g_rssi_buf.head].timestamp = target_ts;
    g_rssi_buf.head = (g_rssi_buf.head + 1) % RSSI_BUF_SIZE;
    if (g_rssi_buf.head == g_rssi_buf.tail) {
      g_rssi_buf.tail = (g_rssi_buf.tail + 1) % RSSI_BUF_SIZE;
    }

    g_rssi_buf.last_timestamp = target_ts;
    target_ts += INTERPOLATION_INTERVAL_US;
  }
}

// Motor Control Task - Pinned to Core 1
static void motor_task(void *pvParameter) {

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1); // 1000Hz

  while (1) {
    vTaskDelayUntil(&last_wake_time, period);

    // --- Update Motor Mixing ---
    // Throttle + Vector
    // Meltybrain math:
    // Motor Power = Throttle + Translation_Mag * cos(angle + Translation_Phase)

    int throttle = g_control_input.throttle;
    // Apply motor command
    // Note: Dshot RMT encoder usage needs to be correct.

    rmt_transmit(esc_chan_a, dshot_encoder_a, &throttle, sizeof(throttle),
                 &((rmt_transmit_config_t){.loop_count = 0}));
    rmt_transmit(esc_chan_b, dshot_encoder_b, &throttle, sizeof(throttle),
                 &((rmt_transmit_config_t){.loop_count = 0}));

    // --- Update LED ---
    // Green in 45 deg arc opposite peak.
    // Needs current 'angle' estimation based on timestamp.
    int64_t now = esp_timer_get_time();
    int64_t time_since_peak = now - g_rotation_state.last_peak_timestamp;
    float phase = 2.0f * M_PI * (float)time_since_peak /
                  g_rotation_state.estimated_period_us;
    // Normalize phase 0..2PI
    phase = fmod(phase, 2.0f * M_PI);

    // Check if in arc (e.g. PI +/- PI/8)
    if (phase > (M_PI - M_PI / 4.0) && phase < (M_PI + M_PI / 4.0)) {
      led_strip_set_pixel(g_led_strip, 0, 0, 20, 0); // Green
    } else {
      led_strip_set_pixel(g_led_strip, 0, 20, 0, 0); // Red
    }
    led_strip_refresh(g_led_strip);
  }
}

// Rotation Estimation Task
static void rotation_task(void *pvParameter) {

  while (1) {
    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(1));

    int head = g_rssi_buf.head;
    int count = (head - g_rssi_buf.tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;

    if (count < CORRELATION_WINDOW * 2)
      continue; // Need enough data

    // --- 1. Autocorrelation (Difference Function) ---
    // We want to find best lag L in range [MIN_PERIOD, MAX_PERIOD]

    int32_t best_lag = 0;
    int64_t min_diff = INT64_MAX;

    // Optimization: Only search around predicted period if we have one?
    int start_lag = 100; // 10ms
    int end_lag = 3000;  // 300ms

    if (g_rotation_state.estimated_period_us > 0) {
      int predicted_lag =
          g_rotation_state.estimated_period_us / INTERPOLATION_INTERVAL_US;
      start_lag = predicted_lag - 200;
      end_lag = predicted_lag + 200;
      if (start_lag < 100)
        start_lag = 100;
      if (end_lag > 5000)
        end_lag = 5000;
    }

    // Subsample for speed
    int step_lag = 5;
    int step_win = 5;

    for (int lag = start_lag; lag <= end_lag; lag += step_lag) {
      int64_t diff_sum = 0;
      for (int i = 0; i < CORRELATION_WINDOW; i += step_win) {
        int idx1 = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
        int idx2 = (head - 1 - i - lag + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
        int8_t v1 = g_rssi_buf.buffer[idx1].rssi;
        int8_t v2 = g_rssi_buf.buffer[idx2].rssi;
        diff_sum += abs(v1 - v2);
      }
      if (diff_sum < min_diff) {
        min_diff = diff_sum;
        best_lag = lag;
      }
    }

    if (best_lag > 0) {
      g_rotation_state.estimated_period_us =
          best_lag * INTERPOLATION_INTERVAL_US;
      g_rotation_state.rotation_rate =
          1000000.0f / g_rotation_state.estimated_period_us;

      // Find Phase Peak in the last period
      int peak_idx = -1;
      int8_t max_rssi = -128;
      for (int i = 0; i < best_lag; i++) {
        int idx = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
        if (g_rssi_buf.buffer[idx].rssi > max_rssi) {
          max_rssi = g_rssi_buf.buffer[idx].rssi;
          peak_idx = idx;
        }
      }
      if (peak_idx >= 0) {
        g_rotation_state.last_peak_timestamp =
            g_rssi_buf.buffer[peak_idx].timestamp;
      }
    }
  }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf || !info->len) {
    return;
  }

  interpolate_rssi(esp_timer_get_time(), info->rx_ctrl.rssi);
}

// Send Callback
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  g_send_status = (status == ESP_NOW_SEND_SUCCESS);
  xSemaphoreGiveFromISR(g_send_cb_sem, NULL);
}

// Task to flush the buffer
// Removed flush_task

// ESP-NOW Receive Callback - PASSIVE, just updates timestamp
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {
  // Extract RSSI
  if (recv_info->rx_ctrl) {
    interpolate_rssi(esp_timer_get_time(), recv_info->rx_ctrl->rssi);
  }

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

  // Parse Control Packet
  // Format: [Throttle (2 bytes)] [VectorX (4 bytes)] [VectorY (4 bytes)] ?
  // User said: "The control state will have a desired throttle, and a
  // desired translation vector." Let's assume binary format for now.

  if (len >= sizeof(uint16_t)) {
    uint16_t throttle;
    memcpy(&throttle, data, sizeof(uint16_t));
    g_control_input.throttle = throttle;
    // Pending: Vector parsing if we define the struct better.
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
  /* Initialize ESPNOW and register sending and receiving callback function.
   */
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

  // DShot Init
  ESP_LOGI(TAG, "Initializing DShot on GPIO %d and %d", DSHOT_ESC_GPIO_NUM_A,
           DSHOT_ESC_GPIO_NUM_B);

  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = 300000,
      .post_delay_us = 50,
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_a));
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_b));

  rmt_tx_channel_config_t tx_chan_config_a = {
      .gpio_num = DSHOT_ESC_GPIO_NUM_A,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_a, &esc_chan_a));

  rmt_tx_channel_config_t tx_chan_config_b = {
      .gpio_num = DSHOT_ESC_GPIO_NUM_B,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_b, &esc_chan_b));

  ESP_ERROR_CHECK(rmt_enable(esc_chan_a));
  ESP_ERROR_CHECK(rmt_enable(esc_chan_b));

  // Start DShot logic
  dshot_esc_throttle_t throttle_val = {.throttle = 0, .telemetry_req = false};
  ESP_ERROR_CHECK(rmt_transmit(esc_chan_a, dshot_encoder_a, &throttle_val,
                               sizeof(throttle_val),
                               &((rmt_transmit_config_t){.loop_count = 0})));
  ESP_ERROR_CHECK(rmt_transmit(esc_chan_b, dshot_encoder_b, &throttle_val,
                               sizeof(throttle_val),
                               &((rmt_transmit_config_t){.loop_count = 0})));

  g_data_mutex = xSemaphoreCreateMutex();

  xTaskCreate(rotation_task, "rotation_task", 4096, NULL, 10, NULL);
  xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 10, NULL, 1);

  example_wifi_init();
  example_espnow_init();
}
