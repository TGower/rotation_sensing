#include <SDL3/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"
#include "implot.h"
#include "SerialPort.h"
#include "GamepadInput.h"
#include "DataLogger.h"
#include <iostream>
#include <deque>
#include <mutex>
#include <cmath>

// Globals
SerialPort serial;
GamepadInput gamepad;
DataLogger logger;

std::mutex state_mtx;
stats_packet_t last_stats = {0};
app_config_packet_t curr_config = {0};
bool config_synced = false;

// Plot Data
const int PLOT_MAX_POINTS = 150; // 15 seconds at 10Hz
std::vector<float> time_data;
std::vector<float> rpm_data;
float t_start = 0;

void OnStats(const stats_packet_t& stats) {
    std::lock_guard<std::mutex> lock(state_mtx);
    last_stats = stats;
    logger.logStats(stats);

    // Rotation rate is in Hz based on memory. RPM = Hz * 60
    float rpm = stats.rotation_rate * 60.0f;

    float current_time = ((float)SDL_GetTicks() / 1000.0f) - t_start;
    time_data.push_back(current_time);
    rpm_data.push_back(rpm);

    if (time_data.size() > PLOT_MAX_POINTS) {
        time_data.erase(time_data.begin());
        rpm_data.erase(rpm_data.begin());
    }
}

void OnConfig(const app_config_packet_t& config) {
    std::lock_guard<std::mutex> lock(state_mtx);
    curr_config = config;
    logger.logConfigState(config);
    config_synced = true;
}

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        return -1;
    }
    SDL_Window* window = SDL_CreateWindow("Deck Meltybrain App", 1280, 720, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Scale for Deck
    ImGui::GetStyle().ScaleAllSizes(1.5f);

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    serial.onStatsReceived(OnStats);
    serial.onConfigReceived(OnConfig);

    logger.start();
    t_start = ((float)SDL_GetTicks() / 1000.0f);

    uint64_t last_tx_time = 0;

    char port_buf[64] = "";

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) done = true;
        }

        serial.update();
        gamepad.update();

        uint64_t now = SDL_GetTicks();
        if (serial.isOpen() && (now - last_tx_time > 50)) { // 20Hz control Tx
            // Map trigger to throttle: 0 to 1000 max (48 min)
            float t = gamepad.getRightTrigger();
            uint16_t throttle = 0;
            if (t > 0.01f) {
                throttle = 48 + (uint16_t)(t * 952.0f);
            }
            float vx = gamepad.getLeftX();
            float vy = -gamepad.getLeftY(); // Invert Y

            serial.writeControl(throttle, vx, vy);

            control_packet_t cp;
            cp.throttle = throttle; cp.vector_x = vx; cp.vector_y = vy;
            logger.logControl(cp);

            last_tx_time = now;
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // --- UI ---
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(1260, 700), ImGuiCond_FirstUseEver);
        ImGui::Begin("Dashboard", nullptr, ImGuiWindowFlags_NoCollapse);

        ImGui::Columns(2);

        // Left Column: Status & Plot
        ImGui::Text("Status: %s", serial.isOpen() ? "CONNECTED" : "DISCONNECTED");
        if (ImGui::InputText("Manual Port Override", port_buf, sizeof(port_buf))) {
            serial.setManualPort(port_buf);
        }

        ImGui::Separator();
        ImGui::Text("Gamepad: %s", gamepad.isConnected() ? "CONNECTED" : "DISCONNECTED");
        ImGui::Text("Throttle: %.2f | Vx: %.2f Vy: %.2f", gamepad.getRightTrigger(), gamepad.getLeftX(), gamepad.getLeftY());

        ImGui::Separator();
        {
            std::lock_guard<std::mutex> lock(state_mtx);
            ImGui::Text("Telemetry:");
            ImGui::Text("RPM: %.1f", last_stats.rotation_rate * 60.0f);
            ImGui::Text("RSSI Mean: %.1f | Var: %.1f | Last: %d", last_stats.rssi_mean, last_stats.rssi_var, (int)last_stats.last_rssi);
            ImGui::Text("Pkts/Sec: %d", last_stats.pkts_per_sec);
        }

        if (ImPlot::BeginPlot("RPM History", ImVec2(-1, 300))) {
            std::lock_guard<std::mutex> lock(state_mtx);
            ImPlot::SetupAxes("Time (s)", "RPM", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            if (time_data.size() > 0) {
                float t_min = time_data.back() - 15.0f;
                float t_max = time_data.back();
                ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Always);
                ImPlot::PlotLine("RPM", time_data.data(), rpm_data.data(), time_data.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::NextColumn();

        // Right Column: Settings
        ImGui::Text("Settings");
        ImGui::Separator();

        std::lock_guard<std::mutex> lock(state_mtx);
        if (config_synced) {
            bool changed = false;

            ImGui::Text("Tuning Config");
            int src = curr_config.rotation_source;
            if (ImGui::Combo("Rotation Source", &src, "CSI\0ESPNOW\0")) {
                curr_config.rotation_source = src; changed = true;
            }

            int s_lag = curr_config.step_lag;
            if (ImGui::SliderInt("Step Lag", &s_lag, 0, 100)) {
                curr_config.step_lag = s_lag; changed = true;
            }
            int s_win = curr_config.step_window;
            if (ImGui::SliderInt("Step Window", &s_win, 0, 100)) {
                curr_config.step_window = s_win; changed = true;
            }

            int c_win = curr_config.correlation_window;
            if (ImGui::SliderInt("Corr Window", &c_win, 0, 100)) {
                curr_config.correlation_window = c_win; changed = true;
            }

            int sm_win = curr_config.smoothing_window;
            if (ImGui::SliderInt("Smooth Window", &sm_win, 0, 100)) {
                curr_config.smoothing_window = sm_win; changed = true;
            }
            if (ImGui::SliderInt("Step Window", &s_win, 0, 100)) {
                curr_config.step_window = s_win; changed = true;
            }

            ImGui::Separator();
            ImGui::Text("Multipliers");

            if (ImGui::DragFloat("Throttle Mult", &curr_config.throttle_multiplier, 0.1f, 0.1f, 10.0f)) changed = true;
            if (ImGui::DragFloat("Translation Mult", &curr_config.translation_multiplier, 0.1f, 0.1f, 10.0f)) changed = true;
            if (ImGui::DragFloat("Phase Offset", &curr_config.phase_offset, 0.1f, -M_PI, M_PI)) changed = true;

            int method = curr_config.translation_method;
            if (ImGui::Combo("Method", &method, "SQUARE\0SINE\0LINEAR\0")) {
                curr_config.translation_method = method; changed = true;
            }

            ImGui::Separator();
            ImGui::Text("Hardware Config (Reboot Required)");
            int da = curr_config.dshot_pin_a;
            if (ImGui::InputInt("DShot Pin A", &da)) { curr_config.dshot_pin_a = da; changed = true; }
            int db = curr_config.dshot_pin_b;
            if (ImGui::InputInt("DShot Pin B", &db)) { curr_config.dshot_pin_b = db; changed = true; }
            int l_pin = curr_config.led_pin;
            if (ImGui::InputInt("LED Pin", &l_pin)) { curr_config.led_pin = l_pin; changed = true; }

            if (changed) {
                serial.writeConfig(curr_config);
                logger.logConfigSet(curr_config);
            }
        } else {
            ImGui::Text("Waiting for configuration sync from device...");
        }

        ImGui::End();

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    logger.stop();
    serial.close();

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
