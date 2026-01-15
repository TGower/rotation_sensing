#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "ota_server";

// --- ESP-NOW Packets (Matches Receiver/Sender) ---
#define APP_PACKET_TYPE_OTA_START 0x50
#define APP_PACKET_TYPE_OTA_DATA 0x51
#define APP_PACKET_TYPE_OTA_ACK 0x52
#define APP_PACKET_TYPE_OTA_END 0x53

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_OTA_START
  uint32_t total_size;
  uint32_t crc32; // Entire file CRC
} ota_start_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_OTA_DATA
  uint16_t seq;
  uint16_t len;
  uint8_t data[200]; // Max 250 payload in ESP-NOW.
} ota_data_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_OTA_ACK
  uint16_t seq; // Sequence number being ACKed
} ota_ack_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_OTA_END
  uint32_t final_crc;
} ota_end_packet_t;

// --- Serial Protocol ---
#define SERIAL_START_BYTE 0xAA
#define SERIAL_CMD_START 0xA0
#define SERIAL_CMD_DATA 0xA1
#define SERIAL_CMD_END 0xA2

#define SERIAL_RESP_ACK 0x06
#define SERIAL_RESP_NACK 0x15

// Globals
static SemaphoreHandle_t g_ack_sem;
static volatile uint16_t g_last_ack_seq = 0xFFFF;
static uint8_t g_target_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                 0xFF, 0xFF, 0xFF};

// --- ESP-NOW Callbacks ---
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  // Can use this to detect if packet left the radio, but we rely on APP ACK
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len) {
  if (len < 1)
    return;
  if (data[0] == APP_PACKET_TYPE_OTA_ACK) {
    if (len >= sizeof(ota_ack_packet_t)) {
      const ota_ack_packet_t *pkt = (const ota_ack_packet_t *)data;
      g_last_ack_seq = pkt->seq;
      xSemaphoreGive(g_ack_sem);
    }
  }
}

static void wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_protocol(
      ESP_IF_WIFI_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
}

static void espnow_init(void) {
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
}

static void update_peer(uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }
  // Remove broadcast if exists (or others) just to be clean, or just add
  // We just add.
  esp_now_peer_info_t peer = {0};
  peer.channel = 1;
  peer.ifidx = ESP_IF_WIFI_STA;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
  esp_now_add_peer(&peer);
}

// --- Serial Task ---
static void send_serial_resp(uint8_t cmd, uint8_t status) {
  uint8_t resp[3] = {SERIAL_START_BYTE, cmd, status};
  fwrite(resp, 1, 3, stdout);
  fflush(stdout);
}

static void serial_task(void *pvParameter) {
  uint8_t rx_byte;
  int state = 0; // 0:SYNC, 1:TYPE, 2:LEN_L, 3:LEN_H, 4:PAYLOAD, 5:CS
  uint8_t cmd_type = 0;
  uint16_t payload_len = 0;
  uint16_t idx = 0;
  uint8_t buffer[512];
  uint8_t checksum = 0;

  while (1) {
    int r = fread(&rx_byte, 1, 1, stdin);
    if (r > 0) {
      switch (state) {
      case 0: // SYNC
        if (rx_byte == SERIAL_START_BYTE) {
          state = 1;
          checksum = 0;
        }
        break;
      case 1: // TYPE
        cmd_type = rx_byte;
        checksum ^= rx_byte;
        state = 2;
        break;
      case 2: // LEN_L
        payload_len = rx_byte;
        checksum ^= rx_byte;
        state = 3;
        break;
      case 3: // LEN_H
        payload_len |= (rx_byte << 8);
        checksum ^= rx_byte;
        state = 4;
        idx = 0;
        if (payload_len == 0)
          state = 5; // No payload
        break;
      case 4: // PAYLOAD
        buffer[idx++] = rx_byte;
        checksum ^= rx_byte;
        if (idx >= payload_len) {
          state = 5;
        }
        break;
      case 5: // CHECKSUM
        if (checksum == rx_byte) {
          // Valid Packet
          // Process
          bool success = false;

          if (cmd_type == SERIAL_CMD_START) {
            // Payload: Total Size (4), MAC (6)
            if (payload_len == 10) {
              uint32_t size = *(uint32_t *)buffer;
              memcpy(g_target_mac, &buffer[4], 6);

              update_peer(g_target_mac);

              ota_start_packet_t pkt;
              pkt.type = APP_PACKET_TYPE_OTA_START;
              pkt.total_size = size;
              pkt.crc32 = 0; // Not using full file CRC yet

              // Clear Semaphore
              xSemaphoreTake(g_ack_sem, 0);
              g_last_ack_seq = 0xFFFF; // Reset

              esp_now_send(g_target_mac, (uint8_t *)&pkt, sizeof(pkt));

              // Wait for ACK
              if (xSemaphoreTake(g_ack_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
                if (g_last_ack_seq == 0xFFFF) // Start ACK magic
                  success = true;
              }
            }
          } else if (cmd_type == SERIAL_CMD_DATA) {
            // Payload: Seq(2), Len(2), Data(...)
            if (payload_len >= 4) {
              uint16_t seq = *(uint16_t *)buffer;
              uint16_t dlen = *(uint16_t *)&buffer[2];

              ota_data_packet_t pkt;
              pkt.type = APP_PACKET_TYPE_OTA_DATA;
              pkt.seq = seq;
              pkt.len = dlen;
              if (dlen > sizeof(pkt.data))
                dlen = sizeof(pkt.data);
              memcpy(pkt.data, &buffer[4], dlen);

              xSemaphoreTake(g_ack_sem, 0);
              esp_now_send(g_target_mac, (uint8_t *)&pkt,
                           sizeof(ota_data_packet_t) - sizeof(pkt.data) + dlen);

              if (xSemaphoreTake(g_ack_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
                if (g_last_ack_seq == seq)
                  success = true;
              }
            }
          } else if (cmd_type == SERIAL_CMD_END) {
            ota_end_packet_t pkt;
            pkt.type = APP_PACKET_TYPE_OTA_END;
            pkt.final_crc = 0;

            xSemaphoreTake(g_ack_sem, 0);
            esp_now_send(g_target_mac, (uint8_t *)&pkt, sizeof(pkt));

            if (xSemaphoreTake(g_ack_sem, pdMS_TO_TICKS(2000)) == pdTRUE) {
              if (g_last_ack_seq == 0xFFFF)
                success = true;
            }
          }

          send_serial_resp(cmd_type,
                           success ? SERIAL_RESP_ACK : SERIAL_RESP_NACK);

        } else {
          ESP_LOGE(TAG, "CS Fail");
        }
        state = 0;
        break;
      }
    } else {
      vTaskDelay(1);
    }
  }
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  g_ack_sem = xSemaphoreCreateBinary();

  wifi_init();
  espnow_init();

  xTaskCreate(serial_task, "serial_task", 4096, NULL, 5, NULL);
}
