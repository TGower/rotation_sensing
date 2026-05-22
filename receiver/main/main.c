/* Receiver Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include "driver/i2c.h"
// #include "driver/usb_serial_jtag.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "led_strip.h"
#include "main.h"
#include "math.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LEDC_IO 12
#define ESPNOW_MAXDELAY 512

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz
#define DSHOT_ESC_GPIO_NUM_A 13
#define DSHOT_ESC_GPIO_NUM_B 9

#define RSSI_BUF_SIZE 6000
#define INTERPOLATION_INTERVAL_US 100
#define LED_STRIP_LED_COUNT 10
#define POV_TEXT_GLYPH_WIDTH 5
#define POV_TEXT_GLYPH_HEIGHT 7
#define POV_TEXT_CHAR_ADVANCE 6
#define POV_TEXT_MARGIN_COLUMNS 4
#define POV_TEXT_TOP_MARGIN 1
#define POV_RSSI_LED_COUNT 9
#define RSSI_DISPLAY_RANGE_HISTORY_COUNT 5

static const char *TAG = "receiver";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF};
static uint8_t g_target_mac[ESP_NOW_ETH_ALEN] = {0};
static bool g_target_mac_set = false;
static SemaphoreHandle_t g_send_cb_sem;
static volatile bool g_send_status = false;

// Assembly function declaration
extern int32_t calculate_sad_vector(const int8_t *A, const int8_t *B, int len,
                                    const int8_t *ones);

// Aligned Vector of Ones (16 bytes)
static const int8_t aligned_ones[16] __attribute__((aligned(16))) = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

typedef struct {
  int8_t rssi[RSSI_BUF_SIZE];
  int64_t timestamp[RSSI_BUF_SIZE];
  int head;
  int tail;
  int64_t last_timestamp;
  int8_t last_rssi;
} rssi_circular_buffer_t;

typedef struct {
  int8_t rssi[RSSI_BUF_SIZE];
  int64_t timestamp[RSSI_BUF_SIZE];
  control_packet_t control[RSSI_BUF_SIZE];
  int head;
  int tail;
  int64_t last_timestamp;
  int8_t last_rssi;
  app_config_packet_t config;
} control_circular_buffer_t;

typedef struct {
  uint16_t throttle;
  float vector_x;
  float vector_y;
} control_input_t;

typedef struct {
  float rotation_rate; // Radians per second? Or generic units? Let's say Hz for
                       // now or normalized.
  float phase_offset;
  int64_t last_peak_timestamp;
  float estimated_period_us;
} rotation_state_t;

typedef struct {
  float phase;
  int64_t last_update_time;
  bool valid;
} dead_reckoning_heading_t;

typedef struct {
  float mean;
  float median;
  float variance;
  int count;
} buf_stats_t;

typedef struct {
  int8_t min;
  int8_t max;
  int count;
  bool valid;
} rssi_range_t;

static rssi_circular_buffer_t g_interpolated_rssi_buf = {0};
static control_circular_buffer_t g_control_buf = {0};
static control_input_t g_control_input = {0};
static rotation_state_t g_rotation_state = {0};
static dead_reckoning_heading_t g_dead_reckoning_heading = {0};
static rssi_range_t g_rssi_display_range = {0};
static rssi_range_t
    g_rssi_display_range_history[RSSI_DISPLAY_RANGE_HISTORY_COUNT] = {0};
static int g_rssi_display_range_history_next = 0;
static int g_rssi_display_range_history_count = 0;

static app_config_packet_t g_config = {
    .type = APP_PACKET_TYPE_CONFIG_STATE,
    .magic = APP_PROTOCOL_MAGIC,
    .dshot_pin_a = DSHOT_ESC_GPIO_NUM_A,
    .dshot_pin_b = DSHOT_ESC_GPIO_NUM_B,
    .led_pin = LEDC_IO,
    .rotation_source = APP_ROTATION_SOURCE_ESPNOW,
    .step_lag = 5,
    .step_window = 5,
    .smoothing_window = 20,
    .throttle_multiplier = 2.0f,
    .translation_multiplier = 4.0f,
    .correlation_window = 1000,
    .translation_method = TRANSLATION_METHOD_SINE,
    .led_display_mode = APP_LED_DISPLAY_MODE_RSSI_POV};

static bool rotation_source_uses_csi(uint8_t rotation_source) {
  return rotation_source == APP_ROTATION_SOURCE_CSI ||
         rotation_source == APP_ROTATION_SOURCE_CSI_DEAD_RECKONING;
}

static bool rotation_source_uses_espnow(uint8_t rotation_source) {
  return rotation_source == APP_ROTATION_SOURCE_ESPNOW ||
         rotation_source == APP_ROTATION_SOURCE_ESPNOW_DEAD_RECKONING;
}

static bool rotation_source_uses_dead_reckoning(uint8_t rotation_source) {
  return rotation_source == APP_ROTATION_SOURCE_CSI_DEAD_RECKONING ||
         rotation_source == APP_ROTATION_SOURCE_ESPNOW_DEAD_RECKONING;
}

static bool app_packet_type_len_is_valid(uint8_t type, int len) {
  switch (type) {
  case APP_PACKET_TYPE_CONTROL:
    return len == sizeof(control_packet_t);
  case APP_PACKET_TYPE_CONFIG_SET:
    return len == sizeof(app_config_packet_t);
  case APP_PACKET_TYPE_CMD_DUMP:
    return len == APP_CMD_DUMP_PACKET_SIZE;
  default:
    return false;
  }
}

static void sanitize_config(void) {
  g_config.magic = APP_PROTOCOL_MAGIC;
  if (g_config.rotation_source > APP_ROTATION_SOURCE_ESPNOW_DEAD_RECKONING) {
    g_config.rotation_source = APP_ROTATION_SOURCE_ESPNOW;
  }
  if (g_config.step_lag == 0) {
    g_config.step_lag = 5;
  }
  if (g_config.correlation_window == 0) {
    g_config.correlation_window = 1000;
  }
  if (g_config.smoothing_window == 0) {
    g_config.smoothing_window = 20;
  }
  if (g_config.throttle_multiplier < 0.001f) {
    g_config.throttle_multiplier = 1.0f;
  }
  if (g_config.translation_multiplier < 0.001f) {
    g_config.translation_multiplier = 1.0f;
  }
  if (g_config.translation_method > TRANSLATION_METHOD_LINEAR) {
    g_config.translation_method = TRANSLATION_METHOD_SINE;
  }
  if (g_config.led_display_mode > APP_LED_DISPLAY_MODE_RSSI_POV) {
    g_config.led_display_mode = APP_LED_DISPLAY_MODE_RSSI_POV;
  }
}

static void save_config(void) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
  } else {
    err = nvs_set_blob(my_handle, "config", &g_config, sizeof(g_config));
    if (err != ESP_OK)
      ESP_LOGE(TAG, "NVS set failed");
    err = nvs_commit(my_handle);
    nvs_close(my_handle);
  }
}

static void send_config_state(void) {
  g_config.type = APP_PACKET_TYPE_CONFIG_STATE;
  g_config.magic = APP_PROTOCOL_MAGIC;
  const uint8_t *dest_mac = g_target_mac_set ? g_target_mac : s_broadcast_mac;
  esp_err_t err = esp_now_send(dest_mac, (uint8_t *)&g_config, sizeof(g_config));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "CONFIG_DATA send failed: %d", err);
  }
}

static void load_config(void) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS Open mismatch, using defaults");
    return;
  }

  bool should_save = false;
  size_t stored_size = 0;
  err = nvs_get_blob(my_handle, "config", NULL, &stored_size);
  if (err != ESP_OK || stored_size == 0) {
    ESP_LOGW(TAG, "NVS Load failed, saving defaults");
    should_save = true;
  } else if (stored_size != sizeof(g_config)) {
    ESP_LOGW(TAG, "NVS config size mismatch (%u bytes), using defaults",
             (unsigned)stored_size);
    should_save = true;
  } else {
    app_config_packet_t loaded = g_config;
    size_t read_size = stored_size;
    err = nvs_get_blob(my_handle, "config", &loaded, &read_size);
    if (err != ESP_OK || read_size != stored_size) {
      ESP_LOGW(TAG, "NVS config read failed, saving defaults");
      should_save = true;
    } else {
      g_config = loaded;
      sanitize_config();
      ESP_LOGI(TAG,
               "Config Loaded: DShot A=%d, B=%d, LED=%d, Rotation Source=%d, "
               "LED Mode=%d",
               g_config.dshot_pin_a, g_config.dshot_pin_b, g_config.led_pin,
               g_config.rotation_source, g_config.led_display_mode);
    }
  }
  nvs_close(my_handle);

  sanitize_config();
  if (should_save) {
    save_config();
  }
}

static SemaphoreHandle_t g_data_mutex;
static volatile uint32_t g_recv_pkt_count = 0;
static volatile int8_t g_last_rssi = 0;

// Dump State
static volatile bool g_req_dump = false;
static volatile bool g_is_dumping = false;
// DShot Handles
static rmt_channel_handle_t esc_chan_a = NULL;
static rmt_channel_handle_t esc_chan_b = NULL;
static rmt_encoder_handle_t dshot_encoder_a = NULL;
static rmt_encoder_handle_t dshot_encoder_b = NULL;
static led_strip_handle_t g_led_strip = NULL;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

static float wrap_2pi(float angle) {
  angle = fmodf(angle, 2.0f * (float)M_PI);
  if (angle < 0.0f) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static bool phase_in_heading_arc(float phase) {
  float diff = phase - (float)M_PI;
  while (diff <= -(float)M_PI) {
    diff += 2.0f * (float)M_PI;
  }
  while (diff > (float)M_PI) {
    diff -= 2.0f * (float)M_PI;
  }
  return fabsf(diff) < ((float)M_PI / 8.0f);
}

static bool phase_in_forward_arc(float phase) {
  float diff = phase;
  while (diff <= -(float)M_PI) {
    diff += 2.0f * (float)M_PI;
  }
  while (diff > (float)M_PI) {
    diff -= 2.0f * (float)M_PI;
  }
  return fabsf(diff) <= ((float)M_PI / 6.0f);
}

static float update_dead_reckoning_heading(int64_t now, float seed_phase,
                                           float period_us) {
  if (!g_dead_reckoning_heading.valid) {
    g_dead_reckoning_heading.phase = seed_phase;
    g_dead_reckoning_heading.last_update_time = now;
    g_dead_reckoning_heading.valid = true;
    return g_dead_reckoning_heading.phase;
  }

  int64_t elapsed_us = now - g_dead_reckoning_heading.last_update_time;
  if (elapsed_us < 0) {
    g_dead_reckoning_heading.phase = seed_phase;
  } else {
    g_dead_reckoning_heading.phase = wrap_2pi(
        g_dead_reckoning_heading.phase +
        (2.0f * (float)M_PI * (float)elapsed_us / period_us));
  }
  g_dead_reckoning_heading.last_update_time = now;
  return g_dead_reckoning_heading.phase;
}

static void set_all_leds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
    led_strip_set_pixel(g_led_strip, i, r, g, b);
  }
  led_strip_refresh(g_led_strip);
}

static const uint8_t *font5x7_rows(char ch) {
  static const uint8_t glyph_blank[POV_TEXT_GLYPH_HEIGHT] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  static const uint8_t glyph_0[POV_TEXT_GLYPH_HEIGHT] = {
      0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
  static const uint8_t glyph_1[POV_TEXT_GLYPH_HEIGHT] = {
      0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
  static const uint8_t glyph_2[POV_TEXT_GLYPH_HEIGHT] = {
      0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
  static const uint8_t glyph_3[POV_TEXT_GLYPH_HEIGHT] = {
      0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
  static const uint8_t glyph_4[POV_TEXT_GLYPH_HEIGHT] = {
      0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
  static const uint8_t glyph_5[POV_TEXT_GLYPH_HEIGHT] = {
      0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
  static const uint8_t glyph_6[POV_TEXT_GLYPH_HEIGHT] = {
      0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
  static const uint8_t glyph_7[POV_TEXT_GLYPH_HEIGHT] = {
      0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
  static const uint8_t glyph_8[POV_TEXT_GLYPH_HEIGHT] = {
      0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
  static const uint8_t glyph_9[POV_TEXT_GLYPH_HEIGHT] = {
      0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C};
  static const uint8_t glyph_m[POV_TEXT_GLYPH_HEIGHT] = {
      0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
  static const uint8_t glyph_p[POV_TEXT_GLYPH_HEIGHT] = {
      0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
  static const uint8_t glyph_r[POV_TEXT_GLYPH_HEIGHT] = {
      0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};

  switch (ch) {
  case '0':
    return glyph_0;
  case '1':
    return glyph_1;
  case '2':
    return glyph_2;
  case '3':
    return glyph_3;
  case '4':
    return glyph_4;
  case '5':
    return glyph_5;
  case '6':
    return glyph_6;
  case '7':
    return glyph_7;
  case '8':
    return glyph_8;
  case '9':
    return glyph_9;
  case 'M':
  case 'm':
    return glyph_m;
  case 'P':
  case 'p':
    return glyph_p;
  case 'R':
  case 'r':
    return glyph_r;
  default:
    return glyph_blank;
  }
}

static bool font5x7_pixel(char ch, int col, int row) {
  if (col < 0 || col >= POV_TEXT_GLYPH_WIDTH || row < 0 ||
      row >= POV_TEXT_GLYPH_HEIGHT) {
    return false;
  }
  const uint8_t *glyph = font5x7_rows(ch);
  return (glyph[row] & (1 << (POV_TEXT_GLYPH_WIDTH - 1 - col))) != 0;
}

static bool text_pixel(const char *text, int col, int row) {
  if (col < 0 || row < 0 || row >= POV_TEXT_GLYPH_HEIGHT) {
    return false;
  }
  int char_index = col / POV_TEXT_CHAR_ADVANCE;
  int char_col = col % POV_TEXT_CHAR_ADVANCE;
  size_t len = strlen(text);
  if (char_index < 0 || char_index >= (int)len ||
      char_col >= POV_TEXT_GLYPH_WIDTH) {
    return false;
  }
  return font5x7_pixel(text[char_index], char_col, row);
}

static void render_simple_angle_leds(float phase) {
  if (phase_in_heading_arc(phase)) {
    set_all_leds(255, 255, 255);
  } else {
    set_all_leds(0, 0, 0);
  }
}

static void render_rpm_leds(float phase) {
  char text[12];
  float rpm_float = fabsf(g_rotation_state.rotation_rate * 60.0f);
  uint32_t rpm = (uint32_t)(rpm_float + 0.5f);
  if (rpm > 99999) {
    rpm = 99999;
  }
  snprintf(text, sizeof(text), "%luRPM", (unsigned long)rpm);

  int text_width = (int)strlen(text) * POV_TEXT_CHAR_ADVANCE - 1;
  int canvas_width = text_width + 2 * POV_TEXT_MARGIN_COLUMNS;
  float display_phase = wrap_2pi(phase - (float)M_PI);
  int canvas_col =
      (int)((display_phase / (2.0f * (float)M_PI)) * canvas_width);
  if (canvas_col >= canvas_width) {
    canvas_col = canvas_width - 1;
  }
  int text_col = canvas_col - POV_TEXT_MARGIN_COLUMNS;

  for (int led = 0; led < LED_STRIP_LED_COUNT; led++) {
    int row = (LED_STRIP_LED_COUNT - 1 - led) - POV_TEXT_TOP_MARGIN;
    bool on = text_pixel(text, text_col, row);
    led_strip_set_pixel(g_led_strip, led, on ? 255 : 0, on ? 255 : 0,
                        on ? 255 : 0);
  }
  led_strip_refresh(g_led_strip);
}

static rgb_t pokeball_pixel(float display_phase, int led_index) {
  float radius = ((float)led_index + 0.5f) / (float)LED_STRIP_LED_COUNT;
  float x = radius * sinf(display_phase);
  float y = radius * cosf(display_phase);
  float abs_y = fabsf(y);
  (void)x;

  if (radius < 0.18f) {
    return (rgb_t){255, 255, 255};
  }
  if (radius < 0.32f) {
    return (rgb_t){0, 0, 0};
  }
  if (abs_y < 0.07f) {
    return (rgb_t){0, 0, 0};
  }
  if (y >= 0.0f) {
    return (rgb_t){255, 255, 255};
  }
  return (rgb_t){255, 0, 0};
}

static void render_pokeball_leds(float phase) {
  float display_phase = wrap_2pi(phase - (float)M_PI);
  for (int led = 0; led < LED_STRIP_LED_COUNT; led++) {
    rgb_t color = pokeball_pixel(display_phase, led);
    led_strip_set_pixel(g_led_strip, led, color.r, color.g, color.b);
  }
  led_strip_refresh(g_led_strip);
}

static int scale_rssi_to_led_index(int8_t rssi) {
  rssi_range_t range = g_rssi_display_range;
  if (!range.valid || range.max <= range.min) {
    return POV_RSSI_LED_COUNT / 2;
  }

  int scaled = ((int)rssi - (int)range.min) * (POV_RSSI_LED_COUNT - 1);
  int denom = (int)range.max - (int)range.min;
  int led_index = (scaled + denom / 2) / denom;

  if (led_index < 0) {
    return 0;
  }
  if (led_index >= POV_RSSI_LED_COUNT) {
    return POV_RSSI_LED_COUNT - 1;
  }
  return led_index;
}

static void render_rssi_pov_leds(float phase) {
  int rssi_led = scale_rssi_to_led_index(g_last_rssi);
  bool heading_led_on = phase_in_forward_arc(phase);

  for (int led = 0; led < LED_STRIP_LED_COUNT; led++) {
    if (led == rssi_led && led < POV_RSSI_LED_COUNT) {
      led_strip_set_pixel(g_led_strip, led, 0, 180, 255);
    } else if (led == LED_STRIP_LED_COUNT - 1) {
      if (heading_led_on) {
        led_strip_set_pixel(g_led_strip, led, 255, 0, 0);
      } else {
        led_strip_set_pixel(g_led_strip, led, 0, 0, 0);
      }
    } else {
      led_strip_set_pixel(g_led_strip, led, 0, 0, 0);
    }
  }
  led_strip_refresh(g_led_strip);
}

static void update_leds(float phase) {
  switch (g_config.led_display_mode) {
  case APP_LED_DISPLAY_MODE_RPM:
    render_rpm_leds(phase);
    break;
  case APP_LED_DISPLAY_MODE_PICTURE:
    render_pokeball_leds(phase);
    break;
  case APP_LED_DISPLAY_MODE_RSSI_POV:
    render_rssi_pov_leds(phase);
    break;
  case APP_LED_DISPLAY_MODE_SIMPLE_ANGLE:
  default:
    render_simple_angle_leds(phase);
    break;
  }
}

// Helper: Interpolate and Add to Circular Buffer
static void interpolate_rssi(rssi_circular_buffer_t *buf, int64_t timestamp,
                             int8_t rssi) {

  // If buffer is empty, just add the first point
  if (buf->last_timestamp == 0) {
    buf->rssi[buf->head] = rssi;
    buf->timestamp[buf->head] = timestamp;
    buf->last_timestamp = timestamp;
    buf->last_rssi = rssi;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    return;
  }

  // Safety: If gap is too large (> 100ms), reset
  if (timestamp - buf->last_timestamp > 100000) {
    buf->last_timestamp = timestamp;
    buf->last_rssi = rssi;
    buf->rssi[buf->head] = rssi;
    buf->timestamp[buf->head] = timestamp;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    if (buf->head == buf->tail) {
      buf->tail = (buf->tail + 1) % RSSI_BUF_SIZE;
    }
    return;
  }

  if (timestamp <= buf->last_timestamp) {
    ESP_LOGW(TAG, "Timestamp out of order");
    return;
  }

  int8_t prev_rssi = buf->last_rssi;
  int64_t prev_ts = buf->last_timestamp;
  int last_idx = (buf->head - 1 + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
  int64_t target_ts = buf->timestamp[last_idx] + INTERPOLATION_INTERVAL_US;

  while (target_ts <= timestamp) {
    // linear interpolate from prev_rssi to rssi
    float ratio = (float)(target_ts - prev_ts) / (float)(timestamp - prev_ts);
    int8_t val = (int8_t)(prev_rssi + (float)(rssi - prev_rssi) * ratio);

    buf->rssi[buf->head] = val;
    buf->timestamp[buf->head] = target_ts;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    if (buf->head == buf->tail) {
      buf->tail = (buf->tail + 1) % RSSI_BUF_SIZE;
    }
    target_ts += INTERPOLATION_INTERVAL_US;
  }
  buf->last_timestamp = timestamp;
  buf->last_rssi = rssi;
}

static int compare_int8(const void *a, const void *b) {
  return (*(int8_t *)a - *(int8_t *)b);
}

static void calculate_stats(rssi_circular_buffer_t *buf, buf_stats_t *stats) {
  int head = buf->head;
  int tail = buf->tail;
  int count = (head - tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;

  if (count == 0) {
    memset(stats, 0, sizeof(buf_stats_t));
    return;
  }

  int sum = 0;
  int limit = (count > 1000) ? 1000 : count;
  int8_t *temp_vals = malloc(limit * sizeof(int8_t));
  if (!temp_vals) {
    memset(stats, 0, sizeof(buf_stats_t));
    return;
  }

  for (int i = 0; i < limit; i++) {
    int idx = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int8_t val = buf->rssi[idx];
    sum += val;
    temp_vals[i] = val;
  }

  stats->count = limit;
  stats->mean = (float)sum / limit;

  float var_sum = 0;
  for (int i = 0; i < limit; i++) {
    float diff = temp_vals[i] - stats->mean;
    var_sum += diff * diff;
  }
  stats->variance = var_sum / limit;

  qsort(temp_vals, limit, sizeof(int8_t), compare_int8);
  if (limit % 2 == 0) {
    stats->median = (temp_vals[limit / 2 - 1] + temp_vals[limit / 2]) / 2.0f;
  } else {
    stats->median = temp_vals[limit / 2];
  }

  free(temp_vals);
}

static void update_rssi_display_range(rssi_circular_buffer_t *buf, int head,
                                      int sample_count) {
  if (sample_count <= 0) {
    g_rssi_display_range.valid = false;
    g_rssi_display_range.count = 0;
    memset(g_rssi_display_range_history, 0,
           sizeof(g_rssi_display_range_history));
    g_rssi_display_range_history_next = 0;
    g_rssi_display_range_history_count = 0;
    return;
  }
  if (sample_count > RSSI_BUF_SIZE) {
    sample_count = RSSI_BUF_SIZE;
  }

  int8_t min_rssi = 127;
  int8_t max_rssi = -128;
  for (int i = 0; i < sample_count; i++) {
    int idx = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int8_t val = buf->rssi[idx];
    if (val < min_rssi) {
      min_rssi = val;
    }
    if (val > max_rssi) {
      max_rssi = val;
    }
  }

  g_rssi_display_range_history[g_rssi_display_range_history_next] =
      (rssi_range_t){.min = min_rssi,
                     .max = max_rssi,
                     .count = sample_count,
                     .valid = true};
  g_rssi_display_range_history_next =
      (g_rssi_display_range_history_next + 1) %
      RSSI_DISPLAY_RANGE_HISTORY_COUNT;
  if (g_rssi_display_range_history_count < RSSI_DISPLAY_RANGE_HISTORY_COUNT) {
    g_rssi_display_range_history_count++;
  }

  int8_t overall_min = 127;
  int8_t overall_max = -128;
  int sample_total = 0;
  bool valid = false;
  for (int i = 0; i < g_rssi_display_range_history_count; i++) {
    rssi_range_t range = g_rssi_display_range_history[i];
    if (!range.valid) {
      continue;
    }
    if (range.min < overall_min) {
      overall_min = range.min;
    }
    if (range.max > overall_max) {
      overall_max = range.max;
    }
    sample_total += range.count;
    valid = true;
  }

  g_rssi_display_range.min = overall_min;
  g_rssi_display_range.max = overall_max;
  g_rssi_display_range.count = sample_total;
  g_rssi_display_range.valid = valid;
}

static void update_recv_stats(int8_t rssi) {
  g_recv_pkt_count++;
  g_last_rssi = rssi;
}

// Motor Control Task - Pinned to Core 1
static void motor_task(void *pvParameter) {

  while (1) {
    vTaskDelay(1);
    // check current heading
    int64_t now = esp_timer_get_time();
    int64_t time_since_peak = now - g_rotation_state.last_peak_timestamp;
    int64_t time_since_last_update = now - g_control_buf.last_timestamp;

    int leftDShot = 0;
    int rightDShot = 0;
    if (time_since_last_update > 1000000) // 1s
    {
      // Stale control data, shut down
      rmt_transmit(esc_chan_a, dshot_encoder_a, &leftDShot, sizeof(leftDShot),
                   &((rmt_transmit_config_t){.loop_count = 0}));
      rmt_transmit(esc_chan_b, dshot_encoder_b, &rightDShot, sizeof(rightDShot),
                   &((rmt_transmit_config_t){.loop_count = 0}));

      set_all_leds(255, 0, 0);
      g_dead_reckoning_heading.valid = false;
      g_dead_reckoning_heading.last_update_time = 0;
      continue;
    } else {
      leftDShot = g_control_input.throttle;
      rightDShot = g_control_input.throttle;
    }

    float period_us = g_rotation_state.estimated_period_us;
    if (period_us < 1.0f) {
      period_us = 2000000.0f;
    }
    float iq_phase = wrap_2pi(2.0f * (float)M_PI *
                              (float)time_since_peak / period_us);
    float selected_phase = iq_phase;
    if (rotation_source_uses_dead_reckoning(g_config.rotation_source)) {
      selected_phase = update_dead_reckoning_heading(now, iq_phase, period_us);
    } else {
      g_dead_reckoning_heading.valid = false;
      g_dead_reckoning_heading.last_update_time = 0;
    }

    // Apply User Offset
    float phase = selected_phase + g_config.phase_offset;

    // Normalize phase 0..2PI
    phase = wrap_2pi(phase);

    const double TRANSLATION_BASE_STRENGTH = 100;

    // --- Update Motor Mixing ---
    // Throttle + Vector
    // Meltybrain math:
    // Motor Power = Throttle + Translation_Mag * cos(angle + Translation_Phase)
    int throttle = g_control_input.throttle;
    if (throttle < 48) {
      // < 48 is DShot command, not sure we are actually handling that correctly
      // in sender app, so default to 0 == STOP command.
      leftDShot = 0;
      rightDShot = 0;
    } else {
      // throttle is a speed
      throttle = (g_control_input.throttle * g_config.throttle_multiplier);

      // Apply multiplied throttle as baseline
      leftDShot = throttle;
      rightDShot = throttle;

      float vx = g_control_input.vector_x;
      float vy = g_control_input.vector_y;
      float mag = sqrtf(vx * vx + vy * vy);

      // If magnitude is significant, apply translation
      if (mag > 0.1f) {
        float target_angle = atan2f(-vy, vx) + M_PI_2;
        // Normalize 0..2PI
        if (target_angle < 0)
          target_angle += 2.0f * M_PI;
        if (target_angle >= 2.0f * M_PI)
          target_angle -= 2.0f * M_PI;

        // Calculate diff in range -PI to PI
        float diff = phase - target_angle;
        while (diff <= -M_PI)
          diff += 2.0f * M_PI;
        while (diff > M_PI)
          diff -= 2.0f * M_PI;

        float strength =
            TRANSLATION_BASE_STRENGTH * g_config.translation_multiplier * mag;

        if (g_config.translation_method == TRANSLATION_METHOD_SQUARE) {
          if (fabsf(diff) < (M_PI / 8.0)) {
            leftDShot = throttle + strength;
            rightDShot = throttle - strength;
          }
        } else if (g_config.translation_method == TRANSLATION_METHOD_SINE) {
          // Sine Wave Modulation
          // Max strength at diff = 0 (cos(0)=1)
          // Zero added at diff = +/- 90 (cos(90)=0)
          // Min strength at diff = +/- 180 (cos(180)=-1)
          float modulation = cosf(diff) * strength;
          leftDShot = throttle + modulation;
          rightDShot = throttle - modulation;

        } else if (g_config.translation_method == TRANSLATION_METHOD_LINEAR) {
          // Linear Ramp
          // Ramp from +strength at 0 to -strength at +/- PI
          // Linear based on phase difference
          float factor = 1.0f - (2.0f * fabsf(diff) / M_PI);
          float modulation = factor * strength;
          leftDShot = throttle + modulation;
          rightDShot = throttle - modulation;
        }
      }

      // clamp to DShot range (TODO: Bidirectional support.)
      if (leftDShot < 48)
        leftDShot = 48;
      if (leftDShot > 1023)
        leftDShot = 1023;
      if (rightDShot < 48)
        rightDShot = 48;
      if (rightDShot > 1023)
        rightDShot = 1023;
    }
    rmt_transmit(esc_chan_a, dshot_encoder_a, &leftDShot, sizeof(leftDShot),
                 &((rmt_transmit_config_t){.loop_count = 0}));
    rmt_transmit(esc_chan_b, dshot_encoder_b, &rightDShot, sizeof(rightDShot),
                 &((rmt_transmit_config_t){.loop_count = 0}));

    update_leds(phase);
  }
}

// SIMD Autocorrelation
static int64_t calculate_autocorr_error(rssi_circular_buffer_t *buf, int head,
                                        int lag, int corr_window) {
  int64_t total_diff = 0;

  int start_idx = (head - corr_window + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
  int cur_idx = start_idx;
  int samples_left = corr_window;

  while (samples_left > 0) {
    // Determine max contiguous length
    int contig_A = RSSI_BUF_SIZE - cur_idx;
    int idx_B = (cur_idx - lag + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int contig_B = RSSI_BUF_SIZE - idx_B;

    int block_len = samples_left;
    if (contig_A < block_len)
      block_len = contig_A;
    if (contig_B < block_len)
      block_len = contig_B;

    // Align A (cur_idx) to 16-byte boundary
    // We check the address &buf->smoothed_rssi[cur_idx]
    while (block_len > 0 && ((uintptr_t)&buf->rssi[cur_idx] & 0xF)) {
      total_diff += abs(buf->rssi[cur_idx] - buf->rssi[idx_B]);

      cur_idx = (cur_idx + 1) % RSSI_BUF_SIZE;
      idx_B = (idx_B + 1) % RSSI_BUF_SIZE;
      block_len--;
      samples_left--;
    }

    // Now A is aligned (or block_len is 0)
    int simd_len = (block_len / 16) * 16;
    if (simd_len > 0) {
      total_diff += calculate_sad_vector(&buf->rssi[cur_idx], &buf->rssi[idx_B],
                                         simd_len, aligned_ones);

      cur_idx = (cur_idx + simd_len) % RSSI_BUF_SIZE;
      idx_B = (idx_B + simd_len) % RSSI_BUF_SIZE;
      block_len -= simd_len;
      samples_left -= simd_len;
    }

    // Handle remainder
    while (block_len > 0) {
      total_diff += abs(buf->rssi[cur_idx] - buf->rssi[idx_B]);
      cur_idx = (cur_idx + 1) % RSSI_BUF_SIZE;
      idx_B = (idx_B + 1) % RSSI_BUF_SIZE;
      block_len--;
      samples_left--;
    }
  }

  return total_diff;
}

static void dump_buffer_to_flash() {

  ESP_LOGI(TAG, "Starting Buffer Dump to Flash...");
  g_is_dumping = true;
  g_req_dump = false; // Clear request

  // Copy Config to Buffer
  g_control_buf.config = g_config;

  const esp_partition_t *part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");

  if (part) {
    // Multi-Dump Logic
    nvs_handle_t my_handle;
    uint32_t dump_index = 0;

    // 1. Get Current Index
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
      nvs_get_u32(my_handle, "dump_index", &dump_index);
    }

    // 2. Calculate Offset (128KB slots)
    // Partition is 2MB = 16 slots of 128KB (0x20000)
    // Buffer size is ~78KB, so we need 128KB slots.
    uint32_t slot_size = 0x20000;
    uint32_t max_slots = part->size / slot_size;
    uint32_t offset = (dump_index % max_slots) * slot_size;

    ESP_LOGI(TAG,
             "Starting Buffer Dump for Slot %" PRIu32 " at Offset 0x%" PRIx32
             "...",
             dump_index, offset);

    // Log Buffer Info
    ESP_LOGI(TAG, "Buffer Head: %d, Tail: %d, Count: %d", g_control_buf.head,
             g_control_buf.tail,
             (g_control_buf.head - g_control_buf.tail + RSSI_BUF_SIZE) %
                 RSSI_BUF_SIZE);

    // 3. Erase ONLY the target slot (64KB -> 128KB)
    // Check if buffer fits in slot
    if (sizeof(control_circular_buffer_t) > slot_size) {
      ESP_LOGE(TAG, "Buffer size (%lu) exceeds slot size (%" PRIu32 ")!",
               (unsigned long)sizeof(control_circular_buffer_t), slot_size);
      g_is_dumping = false;
      g_req_dump = false;
      return;
    }

    esp_partition_erase_range(part, offset, slot_size);

    // 4. Write Data
    /*
      We need to dump the entire circular buffer struct.
    */
    esp_partition_write(part, offset, &g_control_buf,
                        sizeof(control_circular_buffer_t));

    ESP_LOGI(TAG, "Buffer Dumped. Sending ACK.");

    // 5. Increment and Save Index
    dump_index++;
    if (my_handle) {
      nvs_set_u32(my_handle, "dump_index", dump_index);
      nvs_commit(my_handle);
      nvs_close(my_handle);
    }

    // Send ACK
    // We can reuse the sending mechanism but here we just need to send a
    // simple packet. For simplicity, we can't easily call espnow_send from
    // here if we don't have the peer. But we DO have g_target_mac if valid.
    if (g_target_mac_set) {
      uint8_t ack_pkt[APP_CMD_ACK_PACKET_SIZE] = {
          APP_PACKET_TYPE_CMD_ACK, APP_PROTOCOL_MAGIC, 0x00};
      esp_now_send(g_target_mac, ack_pkt, sizeof(ack_pkt));
    }
  } else {
    ESP_LOGE(TAG, "Storage partition not found!");
  }
  g_is_dumping = false;
  g_req_dump = false;
}

// Rotation Estimation Task
static void rotation_task(void *pvParameter) {

  while (1) {
    vTaskDelay(1);

    int head = g_interpolated_rssi_buf.head;
    int count =
        (head - g_interpolated_rssi_buf.tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    uint16_t corr_window = g_config.correlation_window;
    if (corr_window == 0)
      corr_window = 1000; // Safety default

    if (count < corr_window * 2)
      continue; // Need enough data

    // --- Check for Dump Request ---
    if (g_req_dump) {
      dump_buffer_to_flash();
    }

    int range_window = corr_window;
    if (range_window > count) {
      range_window = count;
    }
    update_rssi_display_range(&g_interpolated_rssi_buf, head, range_window);

    // --- 1. Autocorrelation (Difference Function) ---
    // We want to find best lag L in range [MIN_PERIOD, MAX_PERIOD]

    int64_t autocorr_start = esp_timer_get_time();

    int64_t best_lag = 0;
    int64_t max_diff = 0;
    int64_t min_diff = INT64_MAX;

    const int start_lag = 200; // 20ms
    const int end_lag =
        1000; // 100ms This enforces a minimum rotation speed of 600 RPM. If you
              // expect slower rotations, increase this.
#define MAX_LAGS ((1000 - 200) / 5)
    // Subsample for speed
    int step_lag = g_config.step_lag;

    static int64_t errors[MAX_LAGS];
    static int lags[MAX_LAGS];
    int count_lags = 0;

    for (int lag = start_lag; lag < end_lag; lag += step_lag) {
      if (count_lags >= MAX_LAGS)
        break;

      int64_t diff_sum = calculate_autocorr_error(&g_interpolated_rssi_buf,
                                                  head, lag, corr_window);
      errors[count_lags] = diff_sum;
      lags[count_lags] = lag;
      count_lags++;

      if (diff_sum < min_diff) {
        min_diff = diff_sum;
      }
      if (diff_sum > max_diff) {
        max_diff = diff_sum;
      }
    }

    // Global min is often a harmonic, what follows is my huerstic of finding
    // the first "real" dip in the error.

    // Process Slopes & Validate
    best_lag = 0;
    bool found_valid = false;

    // Need at least a few points
    if (count_lags > 3) {
      int64_t max_slope = 0;
      static int64_t slopes[MAX_LAGS]; // slope[i] is from i to i+1

      for (int i = 0; i < count_lags - 1; i++) {
        slopes[i] = errors[i + 1] - errors[i];
        int64_t abs_slope = slopes[i] >= 0 ? slopes[i] : -slopes[i];
        if (abs_slope > max_slope)
          max_slope = abs_slope;
      }

      if (max_slope < 1)
        max_slope = 1;

      // Scan for Zero Crossings
      const int LAG_WINDOW = 1; // Window for d2 check

      for (int i = 0; i < count_lags - 2; i++) {
        // Check for Negative -> Positive Slope (Valley)
        if (slopes[i] < 0 && slopes[i + 1] > 0) {
          int valley_idx = i + 1;

          // 1. Normalized Error Check < 0.5, ensuring that the found valley's
          // error is actually low
          if (2 * (errors[valley_idx] - min_diff) < (max_diff - min_diff)) {
            int64_t d2_sum = 0;
            int count_d2 = 0;

            // 2. Curvature Check (Avg d2), ensuring that the valley has the
            // right shape
            for (int k = i; k >= i - (2 * LAG_WINDOW); k--) {
              if (k < 0 || k >= count_lags - 1)
                continue;
              int64_t d2 = slopes[k + 1] - slopes[k];
              d2_sum += d2;
              count_d2++;
            }

            if (count_d2 > 0) {
              if (20 * d2_sum > count_d2 * max_slope) {
                best_lag = lags[valley_idx];
                found_valid = true;
                break; // Found the first valid one
              }
            }
          }
        }
      }
    }

    int final_lag = best_lag;

    if (!found_valid) {
      // Fallback: 2 seconds, 0.5Hz
      g_rotation_state.estimated_period_us = 2000000;
      g_rotation_state.rotation_rate = 0.5f;
      // Reset valid lag so we don't do fine search on 0
      final_lag = 0;
    } else {
      // Narrow down the best lag, checking +- step_lag
      int64_t fine_min_diff = INT64_MAX;

      for (int i = -step_lag; i <= step_lag; i++) {
        int lag = best_lag + i;
        if (lag < start_lag || lag > end_lag)
          continue;
        int64_t diff_sum = calculate_autocorr_error(&g_interpolated_rssi_buf,
                                                    head, lag, corr_window);
        if (diff_sum < fine_min_diff) {
          fine_min_diff = diff_sum;
          final_lag = lag;
        }
      }
    }

    if (final_lag > 0) {
      g_rotation_state.estimated_period_us =
          final_lag * INTERPOLATION_INTERVAL_US;
      g_rotation_state.rotation_rate =
          1000000.0f / g_rotation_state.estimated_period_us;

      // IQ Demodulation for Phase Tracking. Not entirely happy with this,
      // heading still jumps around sometimes, but is stable for some amount of
      // time. Maybe dead reckoning or something else? Could store a
      // representative sample for the period and do autocorrelation on that to
      // get a stable phase. Window: 4x Period
      float period_us = g_rotation_state.estimated_period_us;
      int window_duration = (int)(4.0f * period_us);

      // Limit window to available data
      if (window_duration > RSSI_BUF_SIZE * INTERPOLATION_INTERVAL_US) {
        window_duration = RSSI_BUF_SIZE * INTERPOLATION_INTERVAL_US;
      }

      int samples_to_process = window_duration / INTERPOLATION_INTERVAL_US;
      if (samples_to_process > RSSI_BUF_SIZE)
        samples_to_process = RSSI_BUF_SIZE;

      double sum_I = 0;
      double sum_Q = 0;
      double omega = 2.0 * M_PI / period_us;

      // Reference time: use the most recent timestamp (head-1)
      // We process backwards from head.
      // Phase phi is relative to cos(omega * (t - t_ref)).
      // t_ref is the timestamp of the HEAD sample (most recent).

      int ref_idx = (head - 1 + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
      int64_t t_ref = g_interpolated_rssi_buf.timestamp[ref_idx];

      for (int i = 0; i < samples_to_process; i++) {
        int idx = (ref_idx - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
        int8_t val = g_interpolated_rssi_buf.rssi[idx];
        int64_t t = g_interpolated_rssi_buf.timestamp[idx];

        double dt = (double)(t - t_ref);
        double angle = omega * dt;

        sum_I += val * cos(angle);
        sum_Q += val * sin(angle);
      }

      // Calculate Phase of the Signal
      double phi = atan2(sum_Q, sum_I);

      // Find time where phase would be PI.
      // omega * (t_target - t_ref) - phi = PI
      // t_target = t_ref + (phi + PI) / omega

      double dt_pi = (phi + M_PI) / omega;

      g_rotation_state.last_peak_timestamp = t_ref + (int64_t)dt_pi;
    }
    int64_t autocorr_end = esp_timer_get_time();
    static uint32_t last_autocorr_time = 0;
    last_autocorr_time = (uint32_t)(autocorr_end - autocorr_start);

    static int log_count = 0;
    // Log stats every 1 second (approx 100 * 10ms)
    if (log_count++ % 100 == 0) {
      static uint32_t last_pkt_count = 0;
      uint32_t curr_pkt_count = g_recv_pkt_count;
      uint32_t diff = curr_pkt_count - last_pkt_count;

      // Calculate and print RSSI stats
      buf_stats_t rssi_stats;
      calculate_stats(&g_interpolated_rssi_buf, &rssi_stats);

      ESP_LOGI(TAG,
               "Stats: %" PRIu32 " pkts/sec | Last RSSI: %d | Throttle: %d",
               diff, g_last_rssi, g_control_input.throttle);
      ESP_LOGI(TAG, "Vector: %f, %f", g_control_input.vector_x,
               g_control_input.vector_y);

      // Send Stats Packet back to Sender
      stats_packet_t stats_pkt = {.type = APP_PACKET_TYPE_STATS,
                                  .magic = APP_PROTOCOL_MAGIC,
                                  .rssi_mean = rssi_stats.mean,
                                  .rssi_var = rssi_stats.variance,
                                  .pkts_per_sec = (int32_t)diff,
                                  .last_rssi = g_last_rssi,
                                  .rotation_rate =
                                      g_rotation_state.rotation_rate,
                                  .vector_x = g_control_input.vector_x,
                                  .vector_y = g_control_input.vector_y,
                                  .autocorrelation_time = last_autocorr_time};

      esp_now_send(s_broadcast_mac, (uint8_t *)&stats_pkt, sizeof(stats_pkt));

      // Broadcast Config State if Idle (Throttle 0). This is used to sync the
      // receiver with the sender.
      if (g_control_input.throttle == 0) {
        send_config_state();
      }

      last_pkt_count = curr_pkt_count;
    }
  }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf || !info->len) {
    return;
  }
  if (!g_target_mac_set ||
      memcmp(g_target_mac, info->mac, ESP_NOW_ETH_ALEN) != 0) {
    return;
  }
  if (rotation_source_uses_csi(g_config.rotation_source)) {
    interpolate_rssi(&g_interpolated_rssi_buf, info->rx_ctrl.timestamp,
                     info->rx_ctrl.rssi);
    update_recv_stats(info->rx_ctrl.rssi);
  }
}

// Send Callback
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  g_send_status = (status == ESP_NOW_SEND_SUCCESS);
  xSemaphoreGiveFromISR(g_send_cb_sem, NULL);
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {

  if (g_is_dumping)
    return; // Prevent overwriting during dump

  if (!recv_info || !data || len < APP_CMD_DUMP_PACKET_SIZE ||
      !recv_info->src_addr || !recv_info->rx_ctrl) {
    return;
  }

  uint8_t packet_type = data[0];
  if (data[1] != APP_PROTOCOL_MAGIC ||
      !app_packet_type_len_is_valid(packet_type, len)) {
    return;
  }

  if (g_target_mac_set &&
      memcmp(g_target_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN) != 0) {
    return;
  }

  // Capture Target MAC only after the frame passes protocol validation.
  if (!g_target_mac_set) {
    memcpy(g_target_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    g_target_mac_set = true;
    ESP_LOGI(TAG, "Discovered Target MAC: " MACSTR, MAC2STR(g_target_mac));

    // Add as peer if not exists
    if (!esp_now_is_peer_exist(g_target_mac)) {
      esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
      if (peer != NULL) {
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, g_target_mac, ESP_NOW_ETH_ALEN);
        esp_err_t add_err = esp_now_add_peer(peer);
        free(peer);
        if (add_err == ESP_OK) {
          ESP_LOGI(TAG, "Added Target as Peer");
        } else {
          ESP_LOGE(TAG, "Failed to add Target peer: %d", add_err);
        }
      }
    }
  }

  // Extract RSSI only from validated packets sent by the selected target.
  if (rotation_source_uses_espnow(g_config.rotation_source)) {
    interpolate_rssi(&g_interpolated_rssi_buf, recv_info->rx_ctrl->timestamp,
                     recv_info->rx_ctrl->rssi);
    update_recv_stats(recv_info->rx_ctrl->rssi);
  }

  // Add raw RSSI to raw buffer
  g_control_buf.rssi[g_control_buf.head] = recv_info->rx_ctrl->rssi;
  g_control_buf.timestamp[g_control_buf.head] = recv_info->rx_ctrl->timestamp;
  g_control_buf.last_timestamp = recv_info->rx_ctrl->timestamp;

  // Parse Packet
  if (packet_type == APP_PACKET_TYPE_CONTROL) {
    const control_packet_t *pkt = (const control_packet_t *)data;
    g_control_input.throttle = pkt->throttle;
    g_control_input.vector_x = pkt->vector_x;
    g_control_input.vector_y = pkt->vector_y;
    g_control_buf.control[g_control_buf.head] = *pkt;
  } else if (packet_type == APP_PACKET_TYPE_CMD_DUMP) {
    ESP_LOGI(TAG, "Received DUMP Command");
    g_req_dump = true;
  } else if (packet_type == APP_PACKET_TYPE_CONFIG_SET) {
    app_config_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    ESP_LOGI(TAG, "Received SET CONFIG");

    bool reboot_needed = false;
    if (pkt.dshot_pin_a != g_config.dshot_pin_a ||
        pkt.dshot_pin_b != g_config.dshot_pin_b ||
        pkt.led_pin != g_config.led_pin) {
      reboot_needed = true;
    }

    // Update Global State
    g_config = pkt;
    // Restore type just in case we need to send it back as STATE
    g_config.type = APP_PACKET_TYPE_CONFIG_STATE;
    sanitize_config();

    save_config();
    ESP_LOGI(TAG, "Applied config: Rotation Source=%d, LED Mode=%d",
             g_config.rotation_source, g_config.led_display_mode);
    send_config_state();

    if (reboot_needed) {
      ESP_LOGW(TAG, "Pin config changed. Rebooting...");
      esp_restart();
    }
  }
  g_control_buf.head = (g_control_buf.head + 1) % RSSI_BUF_SIZE;
  if (g_control_buf.head == g_control_buf.tail) {
    g_control_buf.tail = (g_control_buf.tail + 1) % RSSI_BUF_SIZE;
  }
}
// TODO: test long range mode
// TODO: not really using CSI anymore, could remove
/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(
      ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                          WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

  // Enable Promiscuous mode for CSI on some chips/versions
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  // CSI Config
  wifi_csi_config_t csi_config = {
      .lltf_en = true,
      .htltf_en = true,
      .stbc_htltf2_en = true,
      .ltf_merge_en = false,
      .channel_filter_en = false,
      .manu_scale = true,
      .shift = 2,
  };

  // Try to disable CSI first just in case
  esp_wifi_set_csi(false);

  esp_err_t res = esp_wifi_set_csi_config(&csi_config);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set CSI config: %d", res);
  } else {
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
  }
}

static esp_err_t example_espnow_init(void) {
  /* Initialize ESPNOW and register sending and receiving callback function.
   */
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));
  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

  g_send_cb_sem = xSemaphoreCreateBinary();
  if (g_send_cb_sem == NULL) {
    ESP_LOGE(TAG, "Create send cb sem fail");
    esp_now_deinit();
    return ESP_FAIL;
  }

  /* Set primary master key. */
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

  /* Add broadcast peer information to peer list. */
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  if (peer == NULL) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    esp_now_deinit();
    return ESP_FAIL;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = CONFIG_ESPNOW_CHANNEL;
  peer->ifidx = ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(peer));
  free(peer);

  /* Set global ESPNOW rate to 24Mbps to handle high packet rate */
  esp_err_t err =
      esp_wifi_config_espnow_rate(ESPNOW_WIFI_IF, WIFI_PHY_RATE_24M);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Global rate config error: %d (%s)", err,
             esp_err_to_name(err));
  }

  return ESP_OK;
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting Receiver App...");
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  load_config(); // Load pins from NVS

  led_strip_config_t strip_config = {
      .strip_gpio_num = g_config.led_pin,
      .max_leds = LED_STRIP_LED_COUNT,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
  set_all_leds(8, 8, 8); // ~3% brightness startup marker.

  // DShot Init
  ESP_LOGI(TAG, "Initializing DShot on GPIO %d and %d", g_config.dshot_pin_a,
           g_config.dshot_pin_b);

  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = 300000,
      .post_delay_us = 50,
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_a));
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_b));

  rmt_tx_channel_config_t tx_chan_config_a = {
      .gpio_num = g_config.dshot_pin_a,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_a, &esc_chan_a));

  rmt_tx_channel_config_t tx_chan_config_b = {
      .gpio_num = g_config.dshot_pin_b,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_b, &esc_chan_b));

  ESP_ERROR_CHECK(rmt_enable(esc_chan_a));
  ESP_ERROR_CHECK(rmt_enable(esc_chan_b));

  // Start DShot logic
  dshot_esc_throttle_t throttle_val = {.throttle = 0, .telemetry_req = false};
  ESP_ERROR_CHECK(rmt_transmit(esc_chan_a, dshot_encoder_a, &throttle_val,
                               sizeof(throttle_val),
                               &((rmt_transmit_config_t){.loop_count = 0})));
  ESP_ERROR_CHECK(rmt_transmit(esc_chan_b, dshot_encoder_b, &throttle_val,
                               sizeof(throttle_val),
                               &((rmt_transmit_config_t){.loop_count = 0})));

  g_data_mutex = xSemaphoreCreateMutex();

  xTaskCreate(rotation_task, "rotation_task", 8192, NULL, 10, NULL);
  xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 10, NULL, 1);

  example_wifi_init();
  example_espnow_init();
}
