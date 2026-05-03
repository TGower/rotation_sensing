# Deck Meltybrain App

This application is built for the Steam Deck to control and monitor the ESP32 sender device for the Meltybrain combat robot.

## Features
- **SDL3** for windowing and native Gamepad support (Left Joystick = Translation, Right Trigger = Throttle).
- **Serial Communication** with auto-detection for the Sender unit.
- **Live RPM Plot** via ImPlot showing a 15-second moving window.
- **Live Configuration Editing** via ImGui.
- **Data Logging** automatically saves telemetry and control states to timestamped `.csv` files.

## Build Requirements
- `cmake` >= 3.10
- `g++` (C++17 support)
- `SDL3` libraries

## Build Instructions
1. Clone submodules or external libraries (Dear ImGui and ImPlot) to your system.
2. Update the `IMGUI_DIR` and `IMPLOT_DIR` paths in `CMakeLists.txt` if necessary.
3. Run:
   ```bash
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```
4. Run the `./DeckApp` executable.
