/* Sender Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_random.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "main.h"
#include "nvs_flash.h"
#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static const char *TAG = "sender";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF};

// Shared State - Global Defs
static control_packet_t g_control_curr = {.type = APP_PACKET_TYPE_CONTROL};
static app_config_packet_t g_config_curr = {.type = APP_PACKET_TYPE_CONFIG_SET};
static bool g_config_updated = false;
static bool g_dump_req = false;
static uint8_t g_target_mac[ESP_NOW_ETH_ALEN] = {0}; // Learned from stats
static bool g_target_known = false;

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Ensure the PHY supports higher rates (11g/11n)
  ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B |
                                                            WIFI_PROTOCOL_11G |
                                                            WIFI_PROTOCOL_11N));

  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  // This will override the above if enabled in Kconfig
  ESP_ERROR_CHECK(esp_wifi_set_protocol(
      ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                          WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void example_espnow_send_cb(const uint8_t *mac_addr,
                                   esp_now_send_status_t status) {
  // Optional: Log send status if needed, or keep it minimal for speed
  // ESP_LOGD(TAG, "Last Send Status: %d", status);
}

// OTA State
typedef struct {
  bool active;
  const esp_partition_t *update_partition;
  esp_ota_handle_t update_handle;
  uint32_t total_size;
  uint32_t current_offset;
  uint16_t next_seq;
} ota_state_t;
static ota_state_t g_ota_state = {0};

static void handle_ota_packet(const uint8_t *data, int len,
                              const uint8_t *src_mac) {
  uint8_t type = data[0];
  esp_err_t err;

  // Send ACK helper
  ota_ack_packet_t ack = {.type = APP_PACKET_TYPE_OTA_ACK, .seq = 0};

  if (type == APP_PACKET_TYPE_OTA_START) {
    if (len < sizeof(ota_start_packet_t))
      return;
    const ota_start_packet_t *pkt = (const ota_start_packet_t *)data;

    ESP_LOGI(TAG, "OTA START. Size: %lu", (unsigned long)pkt->total_size);

    g_ota_state.update_partition = esp_ota_get_next_update_partition(NULL);
    if (g_ota_state.update_partition == NULL) {
      ESP_LOGE(TAG, "OTA Passive Partition not found");
      return;
    }

    err = esp_ota_begin(g_ota_state.update_partition, OTA_SIZE_UNKNOWN,
                        &g_ota_state.update_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "OTA Begin failed: %s", esp_err_to_name(err));
      return;
    }

    g_ota_state.active = true;
    g_ota_state.total_size = pkt->total_size;
    g_ota_state.current_offset = 0;
    g_ota_state.next_seq = 0;

    // Ack Start (seq 0 used for start ack)
    ack.seq = 0xFFFF;
    esp_now_send(src_mac, (uint8_t *)&ack, sizeof(ack));

  } else if (type == APP_PACKET_TYPE_OTA_DATA) {
    if (!g_ota_state.active)
      return;
    if (len < sizeof(ota_data_packet_t))
      return;

    const ota_data_packet_t *pkt = (const ota_data_packet_t *)data;

    if (pkt->seq == g_ota_state.next_seq) {
      err = esp_ota_write(g_ota_state.update_handle, pkt->data, pkt->len);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA Write failed: %s", esp_err_to_name(err));
        g_ota_state.active = false; // Abort
        return;
      }
      g_ota_state.current_offset += pkt->len;
      g_ota_state.next_seq++;

      // Log progress occasionally
      if (pkt->seq % 100 == 0) {
        ESP_LOGI(TAG, "OTA Progress: %lu / %lu",
                 (unsigned long)g_ota_state.current_offset,
                 (unsigned long)g_ota_state.total_size);
      }
    } else if (pkt->seq < g_ota_state.next_seq) {
      // Duplicate packet, just ACK again
    } else {
      // Future packet? Ignore/Drop
      return;
    }

    // Send ACK for the received sequence
    ack.seq = pkt->seq;
    esp_now_send(src_mac, (uint8_t *)&ack, sizeof(ack));

  } else if (type == APP_PACKET_TYPE_OTA_END) {
    if (!g_ota_state.active)
      return;
    ESP_LOGI(TAG, "OTA END Received. Validating...");

    err = esp_ota_end(g_ota_state.update_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "OTA End failed: %s", esp_err_to_name(err));
      return;
    }

    err = esp_ota_set_boot_partition(g_ota_state.update_partition);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "OTA Set Boot Partition failed: %s", esp_err_to_name(err));
      return;
    }

    ESP_LOGI(TAG, "OTA Success. Rebooting...");
    ack.seq = 0xFFFF; // Magic for End ACK
    esp_now_send(src_mac, (uint8_t *)&ack, sizeof(ack));

    // Delay to let ACK go out
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
  }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {
  // Check for OTA Packets
  if (data[0] >= APP_PACKET_TYPE_OTA_START &&
      data[0] <= APP_PACKET_TYPE_OTA_END) {
    handle_ota_packet(data, len, recv_info->src_addr);
    return;
  }

  // We expect stats packets from receiver
  if (len >= 1) { // At least type byte
    uint8_t type = data[0];
    const char *label = NULL;

    if (type == APP_PACKET_TYPE_STATS && len == sizeof(stats_packet_t)) {
      label = "STATS_DATA";
    } else if (type == APP_PACKET_TYPE_CONFIG_STATE &&
               len == sizeof(app_config_packet_t)) {
      label = "CONFIG_DATA";
    } else if (type == APP_PACKET_TYPE_CMD_ACK) {
      label = "DUMP_ACK";
    }

    if (label) {
      // Learn Target MAC
      if (!g_target_known && recv_info->src_addr) {
        memcpy(g_target_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN);
        g_target_known = true;

        // Add Peer if needed
        if (!esp_now_is_peer_exist(g_target_mac)) {
          esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
          if (peer != NULL) {
            memset(peer, 0, sizeof(esp_now_peer_info_t));
            peer->channel = CONFIG_ESPNOW_CHANNEL;
            peer->ifidx = ESPNOW_WIFI_IF;
            peer->encrypt = false;
            memcpy(peer->peer_addr, g_target_mac, ESP_NOW_ETH_ALEN);
            esp_now_add_peer(peer);
            free(peer);
          }
        }
      }

      uint8_t buffer[64];
      int idx = 0;
      buffer[idx++] = 0xAB; // Start Byte

      // Payload
      memcpy(&buffer[idx], data, len);
      idx += len;

      uint8_t sum = 0;
      for (int i = 1; i < idx; i++)
        sum ^= buffer[i]; // XOR sum

      buffer[idx++] = sum;

      // Print as Hex String
      char hex_str[128 + 1];
      for (int i = 0; i < idx; i++) {
        sprintf(&hex_str[i * 2], "%02X", buffer[i]);
      }
      hex_str[idx * 2] = 0;

      ESP_LOGI(TAG, "%s: %s", label, hex_str);
    }
  }
}

// Packet Protocol - Serial
#define PACKET_START_BYTE 0xAA
// Max length for our internal buffers
#define MAX_PACKET_LEN 32

typedef enum {
  WAIT_SYNC,
  READ_TYPE,
  READ_PAYLOAD,
  READ_CHECKSUM
} parse_state_t;

static int64_t g_last_packet_time = 0;
static SemaphoreHandle_t g_state_mutex;
static void serial_read_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Serial Reader Task");

  uint8_t rx_byte;
  uint8_t buffer[MAX_PACKET_LEN];
  int buf_idx = 0;
  int expected_len = 0;
  uint8_t packet_type = 0;
  parse_state_t state = WAIT_SYNC;

  while (1) {
    int len = fread(&rx_byte, 1, 1, stdin);
    if (len > 0) {
      switch (state) {
      case WAIT_SYNC:
        if (rx_byte == PACKET_START_BYTE) {
          state = READ_TYPE;
        }
        break;

      case READ_TYPE:
        packet_type = rx_byte;
        buffer[0] = packet_type;
        buf_idx = 1;

        if (packet_type == APP_PACKET_TYPE_CONTROL) {
          expected_len = sizeof(control_packet_t);
        } else if (packet_type == APP_PACKET_TYPE_CONFIG_SET) {
          expected_len = sizeof(app_config_packet_t);
        } else if (packet_type == APP_PACKET_TYPE_CMD_DUMP) {
          expected_len = 1; // Type only, no payload struct
        } else {
          // Invalid type
          state = WAIT_SYNC;
          break;
        }

        if (buf_idx >= expected_len) {
          state = READ_CHECKSUM;
        } else {
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD:
        buffer[buf_idx++] = rx_byte;
        if (buf_idx >= expected_len) {
          state = READ_CHECKSUM;
        }
        break;

      case READ_CHECKSUM:
        uint8_t calc_sum = 0;
        // Checksum of Type + Payload
        for (int i = 0; i < expected_len; i++) {
          calc_sum ^= buffer[i];
        }

        if (calc_sum == rx_byte) {
          if (xSemaphoreTake(g_state_mutex, portMAX_DELAY)) {
            if (packet_type == APP_PACKET_TYPE_CONTROL) {
              memcpy(&g_control_curr, buffer, sizeof(control_packet_t));
              g_last_packet_time = esp_timer_get_time();
            } else if (packet_type == APP_PACKET_TYPE_CONFIG_SET) {
              memcpy(&g_config_curr, buffer, sizeof(app_config_packet_t));
              g_config_updated = true;
            } else if (packet_type == APP_PACKET_TYPE_CMD_DUMP) {
              g_dump_req = true;
            }
            xSemaphoreGive(g_state_mutex);
          }
        } else {
          ESP_LOGW(TAG, "Checksum Fail: Calc 0x%02X vs Rx 0x%02X, Len %d",
                   calc_sum, rx_byte, expected_len);
          ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, expected_len, ESP_LOG_WARN);
        }

        state = WAIT_SYNC;
        break;
      }
    } else {
      vTaskDelay(1);
    }
  }
}

static void espnow_sender_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting ESP-NOW Sender Task (Hot Loop)");

  control_packet_t current_ctrl = {0};
  uint32_t count = 0;

  // Set default type
  g_control_curr.type = APP_PACKET_TYPE_CONTROL;

  while (1) {
    int64_t now = esp_timer_get_time();
    bool send_config = false;
    bool send_dump = false;
    app_config_packet_t cfg_pkt;

    // Check for timeout and copy state
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY)) {
      if (now - g_last_packet_time > 100000000) { // 100 second timeout
        g_control_curr.throttle = 0;
      }
      current_ctrl = g_control_curr;

      if (g_config_updated) {
        send_config = true;
        cfg_pkt = g_config_curr;
        g_config_updated = false;
      }

      if (g_dump_req) {
        send_dump = true;
        g_dump_req = false;
      }
      xSemaphoreGive(g_state_mutex);
    }

    // Ensure Type is correct (sanity)
    current_ctrl.type = APP_PACKET_TYPE_CONTROL;

    // Send Control Broadcast
    esp_now_send(s_broadcast_mac, (uint8_t *)&current_ctrl,
                 sizeof(current_ctrl));

    // Send Config Unicast (if pending)
    if (send_config) {
      uint8_t *dest_mac = g_target_known ? g_target_mac : s_broadcast_mac;

      esp_err_t err =
          esp_now_send(dest_mac, (uint8_t *)&cfg_pkt, sizeof(cfg_pkt));
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent Config Packet to " MACSTR, MAC2STR(dest_mac));
      } else {
        ESP_LOGE(TAG, "Config Send Fail: %d", err);
      }
    }

    if (send_dump) {
      uint8_t type = APP_PACKET_TYPE_CMD_DUMP;
      // Broadcast or Unicast? Broadcast ensures it reaches.
      esp_now_send(s_broadcast_mac, &type, 1);
      ESP_LOGI(TAG, "Sent DUMP Command");
    }

    count++;
    // Yield every 6000 packets
    if (count >= 6000) {
      count = 0;
      vTaskDelay(1);
    }
  }
}

static esp_err_t example_espnow_init(void) {
  /* Initialize ESPNOW and register sending and receiving callback function. */
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));

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

  /* Set global ESPNOW rate to 24Mbps to handle high packet rate */
  esp_err_t err =
      esp_wifi_config_espnow_rate(ESPNOW_WIFI_IF, WIFI_PHY_RATE_24M);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Global rate config error: %d (%s)", err,
             esp_err_to_name(err));
  }

  free(peer);

  g_state_mutex = xSemaphoreCreateMutex();
  xTaskCreate(serial_read_task, "serial_read_task", 4096, NULL, 5, NULL);
  xTaskCreate(espnow_sender_task, "espnow_sender_task", 4096, NULL, 4, NULL);

  return ESP_OK;
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  example_wifi_init();
  example_espnow_init();
}
