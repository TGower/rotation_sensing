#pragma once
#include <stdint.h>

#define PACKET_START_BYTE 0xAA
#define RECV_START_BYTE 0xAB

enum AppPacketType {
    APP_PACKET_TYPE_CONTROL = 0x10,
    APP_PACKET_TYPE_CONFIG_SET = 0x20,
    APP_PACKET_TYPE_CONFIG_STATE = 0x21,
    APP_PACKET_TYPE_STATS = 0x30,
    APP_PACKET_TYPE_CMD_DUMP = 0x40,
    APP_PACKET_TYPE_CMD_ACK = 0x41
};

#pragma pack(push, 1)

struct control_packet_t {
    uint8_t type; // APP_PACKET_TYPE_CONTROL
    uint16_t throttle;
    float vector_x;
    float vector_y;
};

struct app_config_packet_t {
    uint8_t type; // APP_PACKET_TYPE_CONFIG_SET or APP_PACKET_TYPE_CONFIG_STATE

    // Hardware Config
    uint8_t dshot_pin_a;
    uint8_t dshot_pin_b;
    uint8_t led_pin;

    // Tuning Config
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
};

struct stats_packet_t {
    uint8_t type; // APP_PACKET_TYPE_STATS
    float rssi_mean;
    float rssi_var;
    int32_t pkts_per_sec;
    int8_t last_rssi;
    float rotation_rate;
    float vector_x;
    float vector_y;
    uint32_t autocorrelation_time;
};

#pragma pack(pop)
