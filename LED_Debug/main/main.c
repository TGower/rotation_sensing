#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include <stdint.h>

#define LED_STRIP_GPIO 13
#define LED_STRIP_COUNT 10
#define LED_RMT_RESOLUTION_HZ (10 * 1000 * 1000)
#define FADE_DELAY_MS 10
#define STARTUP_BLINK_DELAY_MS 250

static const char *TAG = "LED_Debug";
static led_strip_handle_t g_led_strip = NULL;

static void strip_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  for (uint32_t i = 0; i < LED_STRIP_COUNT; i++) {
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, r, g, b));
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting WS2812B debug on GPIO %d with %d LEDs",
           LED_STRIP_GPIO, LED_STRIP_COUNT);

  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_STRIP_GPIO,
      .max_leds = LED_STRIP_COUNT,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = LED_RMT_RESOLUTION_HZ,
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config,
                                           &g_led_strip));

  ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
  strip_set_rgb(255, 255, 255);
  vTaskDelay(pdMS_TO_TICKS(STARTUP_BLINK_DELAY_MS));
  ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
  vTaskDelay(pdMS_TO_TICKS(STARTUP_BLINK_DELAY_MS));

  while (1) {
    for (uint16_t fade = 0; fade <= 255; fade++) {
      strip_set_rgb((uint8_t)fade, 255, (uint8_t)fade);
      vTaskDelay(pdMS_TO_TICKS(FADE_DELAY_MS));
    }

    for (int fade = 255; fade >= 0; fade--) {
      strip_set_rgb((uint8_t)fade, 255, (uint8_t)fade);
      vTaskDelay(pdMS_TO_TICKS(FADE_DELAY_MS));
    }
  }
}
