# LED_Debug

Minimal ESP-IDF project for debugging a 10 LED WS2812B strip connected to
GPIO 13.

- GPIO is hardcoded to 13.
- LED count is hardcoded to 10.
- LED communication uses `led_strip_new_rmt_device` with the WS2812 model, GRB
  color order, 10 MHz RMT resolution, and DMA disabled.
- The onboard GPIO 48 RGB LED is intentionally not driven.
- Each color update writes all 10 pixels before `led_strip_refresh(...)`.
- On boot, the whole strip blinks white once.
- The loop fades from green `(0, 255, 0)` to white `(255, 255, 255)` and back.
