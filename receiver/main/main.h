/* ESPNOW Packet Protocol */

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include "esp_now.h"
#include <stdint.h>

/* ESPNOW can work in both station and softap mode. It is configured in
 * menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE 6
#define APP_ROTATION_SOURCE_CSI 0
#define APP_ROTATION_SOURCE_ESPNOW 1

// --- Application Protocol ---

typedef enum {
  APP_PACKET_TYPE_CONTROL = 0x10,
  APP_PACKET_TYPE_CONFIG_SET = 0x20,   // Command to Set Config
  APP_PACKET_TYPE_CONFIG_STATE = 0x21, // Report Current Config
  APP_PACKET_TYPE_STATS = 0x30,
  APP_PACKET_TYPE_CMD_DUMP = 0x40, // Command to Dump Buffer
  APP_PACKET_TYPE_CMD_ACK = 0x41,  // Acknowledge Dump
  APP_PACKET_TYPE_OTA_START = 0x50,
  APP_PACKET_TYPE_OTA_DATA = 0x51,
  APP_PACKET_TYPE_OTA_ACK = 0x52,
  APP_PACKET_TYPE_OTA_END = 0x53
} app_packet_type_t;

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

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_CONTROL
  uint16_t throttle;
  float vector_x;
  float vector_y;
} control_packet_t;

// Unified Configuration Structure
typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_CONFIG_SET or APP_PACKET_TYPE_CONFIG_STATE

  // Hardware Config (Requires Reboot)
  uint8_t dshot_pin_a;
  uint8_t dshot_pin_b;
  uint8_t led_pin;

  // Tuning Config (Real-time)
  uint8_t rotation_source; // 0=CSI, 1=ESPNOW
  uint16_t step_lag;
  uint16_t step_window;

  // Multipliers
  float throttle_multiplier;
  float translation_multiplier;
  uint16_t correlation_window;
  uint16_t smoothing_window;
  float phase_offset;
  uint8_t translation_method;
} app_config_packet_t;

#define TRANSLATION_METHOD_SQUARE 0
#define TRANSLATION_METHOD_SINE 1
#define TRANSLATION_METHOD_LINEAR 2

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_STATS
  float rssi_mean;
  float rssi_var;
  int32_t pkts_per_sec;
  int8_t last_rssi;
  float rotation_rate;
  float vector_x;
  float vector_y;
  uint32_t autocorrelation_time;
} stats_packet_t;

// Union for easy sizing/handling
typedef union {
  uint8_t type;
  control_packet_t control;
  app_config_packet_t config;
  stats_packet_t stats;
  ota_start_packet_t ota_start;
  ota_data_packet_t ota_data;
  ota_ack_packet_t ota_ack;
  ota_end_packet_t ota_end;
} app_packet_t;

#endif
