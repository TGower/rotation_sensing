#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include "protocol.h"

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& port_name, int baudrate = 115200);
    void close();
    bool isOpen() const;

    void setAutoConnect(bool enable) { auto_connect = enable; }
    void setManualPort(const std::string& port) { manual_port = port; }

    void writeControl(uint16_t throttle, float vx, float vy);
    void writeConfig(const app_config_packet_t& config);

    // Callbacks
    void onStatsReceived(std::function<void(const stats_packet_t&)> cb) { stats_cb = cb; }
    void onConfigReceived(std::function<void(const app_config_packet_t&)> cb) { config_cb = cb; }

    void update(); // call from main loop

private:
    int fd = -1;
    std::atomic<bool> is_open{false};
    std::atomic<bool> auto_connect{true};
    std::string manual_port;

    std::thread read_thread;
    std::atomic<bool> keep_reading{true};

    std::function<void(const stats_packet_t&)> stats_cb;
    std::function<void(const app_config_packet_t&)> config_cb;

    void readLoop();
    void parseBuffer(const std::vector<uint8_t>& buffer);
    std::vector<uint8_t> parseHexStr(const std::string& hexStr);

    void writePacket(const uint8_t* payload, size_t len);

    // Auto connect logic
    uint64_t last_connect_attempt = 0;
    uint64_t last_rx_time = 0;
    std::string findPort();
};
