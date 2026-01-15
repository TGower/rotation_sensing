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

// --- Application Protocol ---

typedef enum {
  APP_PACKET_TYPE_CONTROL = 0x10,
  APP_PACKET_TYPE_CONFIG_SET = 0x20,   // Command to Set Config
  APP_PACKET_TYPE_CONFIG_STATE = 0x21, // Report Current Config
  APP_PACKET_TYPE_STATS = 0x30
} app_packet_type_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_CONTROL
  uint16_t throttle;
  float vector_x;
  float vector_y;
  uint8_t control_flags;
} control_packet_t;

#define CONTROL_FLAG_RESET_PHASE_LOCK (1 << 0)

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
} app_config_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_STATS
  float csi_mean;
  float csi_var;
  float espnow_mean;
  float espnow_var;
  int32_t pkts_per_sec;
  int8_t last_rssi;
  // New Fields
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
} app_packet_t;

#endif
