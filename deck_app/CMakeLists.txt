cmake_minimum_required(VERSION 3.10)
project(DeckApp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL3 REQUIRED)

set(IMGUI_DIR /home/jules/imgui)
set(IMPLOT_DIR /home/jules/implot)

set(SOURCES
    src/main.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer3.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
)

add_executable(DeckApp ${SOURCES})

target_include_directories(DeckApp PRIVATE
    include
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
)

target_link_libraries(DeckApp PRIVATE SDL3::SDL3)
target_sources(DeckApp PRIVATE src/SerialPort.cpp)
target_sources(DeckApp PRIVATE src/GamepadInput.cpp)
target_sources(DeckApp PRIVATE src/DataLogger.cpp)
