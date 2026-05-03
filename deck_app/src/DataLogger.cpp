#include "DataLogger.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

DataLogger::DataLogger() {}

DataLogger::~DataLogger() {
    stop();
}

void DataLogger::start() {
    std::lock_guard<std::mutex> lock(mtx);
    if (is_logging) return;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "telemetry_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    file.open(oss.str());
    if (file.is_open()) {
        file << "timestamp_ms,type,throttle,vx,vy,rssi_mean,rssi_var,pkts_per_sec,last_rssi,rotation_rate,autocorr_time\n";
        is_logging = true;
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "Started logging to " << oss.str() << std::endl;
    }
}

void DataLogger::stop() {
    std::lock_guard<std::mutex> lock(mtx);
    if (is_logging) {
        file.close();
        is_logging = false;
        std::cout << "Stopped logging." << std::endl;
    }
}

void DataLogger::logStats(const stats_packet_t& stats) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!is_logging) return;

    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    file << (now - start_time) << ","
         << "STATS,0,0,0,"
         << stats.rssi_mean << ","
         << stats.rssi_var << ","
         << stats.pkts_per_sec << ","
         << (int)stats.last_rssi << ","
         << stats.rotation_rate << ","
         << stats.autocorrelation_time << "\n";
}

void DataLogger::logControl(const control_packet_t& control) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!is_logging) return;

    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    file << (now - start_time) << ","
         << "CONTROL,"
         << control.throttle << ","
         << control.vector_x << ","
         << control.vector_y << ",0,0,0,0,0,0\n";
}

void DataLogger::logConfigSet(const app_config_packet_t& config) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!is_logging) return;
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    file << (now - start_time) << ","
         << "CONFIG_SET," << config.throttle_multiplier << "," << config.translation_multiplier << "," << config.phase_offset << ",0,0,0,0,0,0\n";
}

void DataLogger::logConfigState(const app_config_packet_t& config) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!is_logging) return;
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    file << (now - start_time) << ","
         << "CONFIG_STATE," << config.throttle_multiplier << "," << config.translation_multiplier << "," << config.phase_offset << ",0,0,0,0,0,0\n";
}
