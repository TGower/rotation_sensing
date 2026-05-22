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
#define APP_PROTOCOL_MAGIC 164
#define APP_ROTATION_SOURCE_CSI 0
#define APP_ROTATION_SOURCE_ESPNOW 1
#define APP_ROTATION_SOURCE_CSI_DEAD_RECKONING 2
#define APP_ROTATION_SOURCE_ESPNOW_DEAD_RECKONING 3

#define APP_LED_DISPLAY_MODE_SIMPLE_ANGLE 0
#define APP_LED_DISPLAY_MODE_RPM 1
#define APP_LED_DISPLAY_MODE_PICTURE 2
#define APP_LED_DISPLAY_MODE_RSSI_POV 3

// --- Application Protocol ---

typedef enum {
  APP_PACKET_TYPE_CONTROL = 0x10,
  APP_PACKET_TYPE_CONFIG_SET = 0x20,   // Command to Set Config
  APP_PACKET_TYPE_CONFIG_STATE = 0x21, // Report Current Config
  APP_PACKET_TYPE_STATS = 0x30,
  APP_PACKET_TYPE_CMD_DUMP = 0x40, // Command to Dump Buffer
  APP_PACKET_TYPE_CMD_ACK = 0x41   // Acknowledge Dump
} app_packet_type_t;

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_CONTROL
  uint8_t magic;
  uint16_t throttle;
  float vector_x;
  float vector_y;
} control_packet_t;

// Unified Configuration Structure
typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_CONFIG_SET or APP_PACKET_TYPE_CONFIG_STATE
  uint8_t magic;

  // Hardware Config (Requires Reboot)
  uint8_t dshot_pin_a;
  uint8_t dshot_pin_b;
  uint8_t led_pin;

  // Tuning Config (Real-time)
  // 0=CSI, 1=ESPNOW, 2=CSI_DEAD_RECKONING, 3=ESPNOW_DEAD_RECKONING
  uint8_t rotation_source;
  uint16_t step_lag;
  uint16_t step_window;

  // Multipliers
  float throttle_multiplier;
  float translation_multiplier;
  uint16_t correlation_window;
  uint16_t smoothing_window;
  float phase_offset;
  uint8_t translation_method;
  uint8_t led_display_mode;
} app_config_packet_t;

#define TRANSLATION_METHOD_SQUARE 0
#define TRANSLATION_METHOD_SINE 1
#define TRANSLATION_METHOD_LINEAR 2

#define APP_CONFIG_PACKET_SIZE 28
_Static_assert(sizeof(app_config_packet_t) == APP_CONFIG_PACKET_SIZE,
               "app_config_packet_t size mismatch! Check alignment/packing.");
_Static_assert(sizeof(control_packet_t) == 12,
               "control_packet_t size mismatch!");

#define APP_CMD_DUMP_PACKET_SIZE 2
#define APP_CMD_ACK_PACKET_SIZE 3

typedef struct __attribute__((packed)) {
  uint8_t type; // APP_PACKET_TYPE_STATS
  uint8_t magic;
  float rssi_mean;
  float rssi_var;
  int32_t pkts_per_sec;
  int8_t last_rssi;
  float rotation_rate;
  float vector_x;
  float vector_y;
  uint32_t autocorrelation_time;
} stats_packet_t;

#define APP_STATS_PACKET_SIZE 31
_Static_assert(sizeof(stats_packet_t) == APP_STATS_PACKET_SIZE,
               "stats_packet_t size mismatch!");

// Union for easy sizing/handling
typedef union {
  uint8_t type;
  control_packet_t control;
  app_config_packet_t config;
  stats_packet_t stats;
} app_packet_t;

#endif
