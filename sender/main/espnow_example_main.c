/* Sender Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "espnow_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
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

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {
  // Log the received data to serial
  // We assume the data received is the csi_log_packet_t from the receiver
  if (len > 0) {
    fwrite(data, 1, len, stdout);
    fflush(stdout);
  }
}

static void sender_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting 0.5ms Sender with Busy Wait");

  esp_task_wdt_add(NULL);

  int64_t next_send_time = esp_timer_get_time();
  const int64_t interval_us = 500; // 0.5ms
  uint32_t count = 0;
  uint32_t errors = 0;
  int64_t last_report_time = next_send_time;

  while (1) {
    int64_t now = esp_timer_get_time();

    // if (now < next_send_time) {
    //   continue;
    // }

    esp_err_t result =
        esp_now_send(s_broadcast_mac, (uint8_t *)&now, sizeof(now));

    if (result != ESP_OK) {
      errors++;
    }
    count++;

    // Report every 2000 iterations (~1 second)
    if (count >= 2000) {
      int64_t duration = now - last_report_time;
      ESP_LOGI(TAG,
               "Stats: %" PRIu32 " pkts, %" PRIu32 " errs, avg intv: %" PRIi64
               " us",
               count, errors, duration / count);
      count = 0;
      errors = 0;
      last_report_time = now;

      // Yield to let IDLE task run and reset its WDT
      vTaskDelay(1);
    }

    next_send_time += interval_us;
    esp_task_wdt_reset();
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

  xTaskCreate(sender_task, "sender_task", 4096, NULL, 4, NULL);

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
