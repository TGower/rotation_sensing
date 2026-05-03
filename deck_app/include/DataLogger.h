#pragma once
#include <fstream>
#include <string>
#include "protocol.h"
#include <mutex>
#include <vector>

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void start();
    void stop();

    void logStats(const stats_packet_t& stats);
    void logControl(const control_packet_t& control);
    void logConfigSet(const app_config_packet_t& config);
    void logConfigState(const app_config_packet_t& config);


private:
    std::ofstream file;
    std::mutex mtx;
    bool is_logging = false;
    uint64_t start_time = 0;
};
