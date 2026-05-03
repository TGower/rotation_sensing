#include "SerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <cstring>

namespace fs = std::filesystem;

SerialPort::SerialPort() {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port_name, int baudrate) {
    last_rx_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    close();

    fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }

    is_open = true;
    keep_reading = true;
    read_thread = std::thread(&SerialPort::readLoop, this);

    std::cout << "Opened " << port_name << std::endl;
    return true;
}

void SerialPort::close() {
    if (is_open) {
        keep_reading = false;
        if (read_thread.joinable()) {
            read_thread.join();
        }
        ::close(fd);
        is_open = false;
        fd = -1;
    }
}

bool SerialPort::isOpen() const {
    return is_open;
}

void SerialPort::writePacket(const uint8_t* payload, size_t len) {
    if (!is_open) return;

    std::vector<uint8_t> pkt;
    pkt.push_back(PACKET_START_BYTE);
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        pkt.push_back(payload[i]);
        checksum ^= payload[i];
    }
    pkt.push_back(checksum);

    ::write(fd, pkt.data(), pkt.size());
}

void SerialPort::writeControl(uint16_t throttle, float vx, float vy) {
    control_packet_t pkt;
    pkt.type = APP_PACKET_TYPE_CONTROL;
    pkt.throttle = throttle;
    pkt.vector_x = vx;
    pkt.vector_y = vy;
    writePacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void SerialPort::writeConfig(const app_config_packet_t& config) {
    app_config_packet_t pkt = config;
    pkt.type = APP_PACKET_TYPE_CONFIG_SET;
    writePacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

std::vector<uint8_t> SerialPort::parseHexStr(const std::string& hexStr) {
    std::vector<uint8_t> out;
    std::string clean;
    for (char c : hexStr) {
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
            clean += c;
        }
    }
    if (clean.length() % 2 != 0) clean.pop_back();
    for (size_t i = 0; i < clean.length(); i += 2) {
        out.push_back(std::stoi(clean.substr(i, 2), nullptr, 16));
    }
    return out;
}

void SerialPort::parseBuffer(const std::vector<uint8_t>& buf) {
    last_rx_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if (buf.size() < 3) return;
    if (buf[0] != RECV_START_BYTE) return;

    uint8_t sum = 0;
    for (size_t i = 1; i < buf.size() - 1; i++) {
        sum ^= buf[i];
    }
    if (sum != buf.back()) {
        std::cout << "Checksum fail!" << std::endl;
        return; // Checksum fail
    }

    uint8_t type = buf[1];
    if (type == APP_PACKET_TYPE_STATS && buf.size() - 2 == sizeof(stats_packet_t)) {
        if (stats_cb) {
            stats_packet_t pkt;
            memcpy(&pkt, &buf[1], sizeof(stats_packet_t));
            stats_cb(pkt);
        }
    } else if (type == APP_PACKET_TYPE_CONFIG_STATE && buf.size() - 2 == sizeof(app_config_packet_t)) {
        if (config_cb) {
            app_config_packet_t pkt;
            memcpy(&pkt, &buf[1], sizeof(app_config_packet_t));
            config_cb(pkt);
        }
    }
}

void SerialPort::readLoop() {
    char buf[1024];
    std::string line_accum;

    while (keep_reading) {
        int n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (buf[i] == '\n') {
                    if (line_accum.find("STATS_DATA:") != std::string::npos ||
                        line_accum.find("CONFIG_DATA:") != std::string::npos) {

                        size_t pos = line_accum.find(": ");
                        if (pos != std::string::npos) {
                            std::string hexStr = line_accum.substr(pos + 2);
                            auto pktBuf = parseHexStr(hexStr);
                            parseBuffer(pktBuf);
                        }
                    }
                    line_accum.clear();
                } else if (buf[i] != '\r') {
                    line_accum += buf[i];
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

std::string SerialPort::findPort() {
    for (const auto& entry : fs::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        if (path.find("ttyACM") != std::string::npos || path.find("ttyUSB") != std::string::npos) {
            return path;
        }
    }
    return "";
}

void SerialPort::update() {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if (is_open && (now - last_rx_time > 2000)) { // 2s timeout
        std::cout << "Connection timed out, closing..." << std::endl;
        close();
    }

    if (auto_connect && !is_open && (now - last_connect_attempt > 2000)) {
        last_connect_attempt = now;
        std::string target = manual_port.empty() ? findPort() : manual_port;
        if (!target.empty()) {
            open(target);
        }
    }
}
