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
#include "espnow_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "led_strip.h"
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

#define LEDC_IO 48
#define ESPNOW_MAXDELAY 512

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz
#define DSHOT_ESC_GPIO_NUM_A 8
#define DSHOT_ESC_GPIO_NUM_B 9

#define RSSI_BUF_SIZE 6000
#define INTERPOLATION_INTERVAL_US 100
// #define CORRELATION_WINDOW 1000 // Moving to config

static const char *TAG = "receiver";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF};
static uint8_t g_target_mac[ESP_NOW_ETH_ALEN] = {0};
static bool g_target_mac_set = false;
static SemaphoreHandle_t g_send_cb_sem;
static volatile bool g_send_status = false;

// I2C Configuration for TMAG5273
#define I2C_MASTER_SCL_IO 12        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 11        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define TMAG5273_ADDR 0x22 // Found Address (TMAG5273A2)

// Registers from Datasheet
#define TMAG_REG_DEVICE_CONFIG_1 0x00
#define TMAG_REG_DEVICE_CONFIG_2 0x01
#define TMAG_REG_SENSOR_CONFIG_1 0x02
#define TMAG_REG_MAN_ID_LSB 0x0E
#define TMAG_REG_MAN_ID_MSB 0x0F
#define TMAG_REG_RESULT_X 0x12 // X_MSB_RESULT

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
  float mean;
  float median;
  float variance;
  int count;
} buf_stats_t;

static rssi_circular_buffer_t g_interpolated_rssi_buf = {0};
static rssi_circular_buffer_t g_raw_rssi_buf = {0};
static control_input_t g_control_input = {0};
static rotation_state_t g_rotation_state = {0};

static app_config_packet_t g_config = {.type = APP_PACKET_TYPE_CONFIG_STATE,
                                       .dshot_pin_a = DSHOT_ESC_GPIO_NUM_A,
                                       .dshot_pin_b = DSHOT_ESC_GPIO_NUM_B,
                                       .led_pin = LEDC_IO,
                                       .rotation_source =
                                           APP_ROTATION_SOURCE_ESPNOW,
                                       .step_lag = 5,
                                       .step_window = 5,
                                       .smoothing_window = 20,
                                       .throttle_multiplier = 2.0f,
                                       .translation_multiplier = 4.0f,
                                       .correlation_window = 1000};

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

static void load_config(void) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS Open mismatch, using defaults");
    return;
  }
  size_t required_size = sizeof(g_config);
  err = nvs_get_blob(my_handle, "config", &g_config, &required_size);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS Load failed, saving defaults");
    save_config();
  } else {
    ESP_LOGI(TAG, "Config Loaded: DShot A=%d, B=%d, LED=%d",
             g_config.dshot_pin_a, g_config.dshot_pin_b, g_config.led_pin);
    // Safety check for multipliers (avoid divide by zero or zero throttle if
    // unitialized)
    if (g_config.throttle_multiplier < 0.001f)
      g_config.throttle_multiplier = 1.0f;
    if (g_config.translation_multiplier < 0.001f)
      g_config.translation_multiplier = 1.0f;
  }
  nvs_close(my_handle);
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

// Helper: Interpolate and Add to Circular Buffer
static void interpolate_rssi(rssi_circular_buffer_t *buf, int64_t timestamp,
                             int8_t rssi) {

  if (g_is_dumping)
    return; // Prevent overwriting during dump

  // Add raw RSSI to raw buffer
  g_raw_rssi_buf.rssi[g_raw_rssi_buf.head] = rssi;
  g_raw_rssi_buf.timestamp[g_raw_rssi_buf.head] = timestamp;
  g_raw_rssi_buf.last_timestamp = timestamp;
  g_raw_rssi_buf.head = (g_raw_rssi_buf.head + 1) % RSSI_BUF_SIZE;
  if (g_raw_rssi_buf.head == g_raw_rssi_buf.tail) {
    g_raw_rssi_buf.tail = (g_raw_rssi_buf.tail + 1) % RSSI_BUF_SIZE;
  }

  // If buffer is empty, just add the first point
  if (buf->last_timestamp == 0) {
    buf->rssi[buf->head] = rssi;
    buf->timestamp[buf->head] = timestamp;
    buf->last_timestamp = timestamp;
    buf->last_rssi = rssi;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    // Tail stays 0 until full? Or just 0.
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
  // Create a temp array for median calculation to avoid modifying main buffer
  // or complex logic Just take last N samples or all valid samples. The buffer
  // is large (6000), let's just take last 1000 or all.
  int limit = (count > 1000) ? 1000 : count;
  int8_t *temp_vals = malloc(limit * sizeof(int8_t));
  if (!temp_vals) {
    // Allocation failed, return empty stats
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

  // Variance
  float var_sum = 0;
  for (int i = 0; i < limit; i++) {
    float diff = temp_vals[i] - stats->mean;
    var_sum += diff * diff;
  }
  stats->variance = var_sum / limit;

  // Median
  qsort(temp_vals, limit, sizeof(int8_t), compare_int8);
  if (limit % 2 == 0) {
    stats->median = (temp_vals[limit / 2 - 1] + temp_vals[limit / 2]) / 2.0f;
  } else {
    stats->median = temp_vals[limit / 2];
  }

  free(temp_vals);
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
    float phase = 2.0f * M_PI * (float)time_since_peak /
                  g_rotation_state.estimated_period_us;

    // Apply User Offset
    phase += g_config.phase_offset;

    // Normalize phase 0..2PI
    phase = fmod(phase, 2.0f * M_PI);
    if (phase < 0)
      phase += 2.0f * M_PI;

    // Check if in arc (e.g. PI +/- PI/8)
    const double HEADING_START = M_PI - M_PI / 8.0;
    const double HEADING_END = M_PI + M_PI / 8.0;
    const double TRANSLATION_BASE_STRENGTH = 100;

    // --- Update Motor Mixing ---
    // Throttle + Vector
    // Meltybrain math:
    // Motor Power = Throttle + Translation_Mag * cos(angle + Translation_Phase)
    int throttle = g_control_input.throttle;
    int leftDShot = throttle;
    int rightDShot = throttle;
    if (throttle < 48) {
      // < 48 is DShot command, not sure we are actually handling that correctly
      // in sender app, so default to 0 == STOP command.
      leftDShot = 0;
      rightDShot = 0;
    } else {
      // throttle is a speed
      throttle = (g_control_input.throttle * g_config.throttle_multiplier);

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

        // Calculate diff
        float diff = phase - target_angle;
        while (diff <= -M_PI)
          diff += 2.0f * M_PI;
        while (diff > M_PI)
          diff -= 2.0f * M_PI;

        if (fabsf(diff) < (M_PI / 8.0)) {
          float strength =
              TRANSLATION_BASE_STRENGTH * g_config.translation_multiplier * mag;
          leftDShot = throttle + strength;
          rightDShot = throttle - strength;
        }
      }

      // clamp to DShot range
      if (leftDShot < 48)
        leftDShot = 48;
      if (leftDShot > 2047)
        leftDShot = 2047;
      if (rightDShot < 48)
        rightDShot = 48;
      if (rightDShot > 2047)
        rightDShot = 2047;
    }
    rmt_transmit(esc_chan_a, dshot_encoder_a, &leftDShot, sizeof(leftDShot),
                 &((rmt_transmit_config_t){.loop_count = 0}));
    rmt_transmit(esc_chan_b, dshot_encoder_b, &rightDShot, sizeof(rightDShot),
                 &((rmt_transmit_config_t){.loop_count = 0}));

    // --- Update LED ---
    // User Request:
    // Green for 45 deg arc opposite peak (Heading).
    // Gradient Blue -> Red -> White for the rest.
    // "Incoming" (0 to PI): Blue -> Red
    // "Outgoing" (PI to 2PI): Red -> White
    // Green Overrides.

    uint8_t r = 0, g = 0, b = 0;
    uint8_t max_intensity = 255;

    // 1. Calculate Base Gradient
    if (phase < M_PI) {
      // Incoming: Blue (0,0,255) -> Red (255,0,0)
      float ratio = phase / M_PI; // 0.0 to 1.0
      r = (uint8_t)(max_intensity * ratio);
      g = 0;
      b = (uint8_t)(max_intensity * (1.0f - ratio));
    } else {
      // Outgoing: Red (255,0,0) -> White (255,255,255)
      float ratio = (phase - M_PI) / M_PI; // 0.0 to 1.0
      r = max_intensity;
      g = (uint8_t)(max_intensity * ratio);
      b = (uint8_t)(max_intensity * ratio);
    }

    // 2. Override with Green Arc (Heading)
    // HEADING_START/END are roughly PI +/- PI/8
    if (phase > HEADING_START && phase < HEADING_END) {
      // Use a bright green, maybe slightly dimmed to match intensity if needed,
      // but standard Green (0, 255, 0) is requested.
      r = 0;
      g = max_intensity;
      b = 0;
    }

    led_strip_set_pixel(g_led_strip, 0, r, g, b);
    led_strip_refresh(g_led_strip);
  }
}

// Rotation Estimation Task
// Helper for Autocorrelation Error Calculation
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
      ESP_LOGI(TAG, "Starting Buffer Dump to Flash...");
      g_is_dumping = true;
      g_req_dump = false; // Clear request

      const esp_partition_t *part = esp_partition_find_first(
          ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");

      // Note: Subtype might be 0xFF (ANY does not work well depending on
      // implementation if type is DATA). In csv we used 0xff (undefined) which
      // corresponds to ESP_PARTITION_SUBTYPE_ANY? Actually 0xff is explicit
      // undefined. Let's try SUBTYPE_ANY.
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
                 "Starting Buffer Dump for Slot %" PRIu32
                 " at Offset 0x%" PRIx32 "...",
                 dump_index, offset);

        // Log Buffer Info
        ESP_LOGI(TAG, "Buffer Head: %d, Tail: %d, Count: %d",
                 g_raw_rssi_buf.head, g_raw_rssi_buf.tail,
                 (g_raw_rssi_buf.head - g_raw_rssi_buf.tail + RSSI_BUF_SIZE) %
                     RSSI_BUF_SIZE);

        // 3. Erase ONLY the target slot (64KB)
        esp_partition_erase_range(part, offset, slot_size);

        // 4. Write Data
        /*
          We need to dump the entire circular buffer struct.
          Size is approx sizeof(rssi_circular_buffer_t).
          Our buffer size fits within 64KB (approx 60KB).
        */
        esp_partition_write(part, offset, &g_raw_rssi_buf,
                            sizeof(rssi_circular_buffer_t));

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
          uint8_t ack_pkt[2] = {APP_PACKET_TYPE_CMD_ACK, 0x00};
          esp_now_send(g_target_mac, ack_pkt, 2);
        }
      } else {
        ESP_LOGE(TAG, "Storage partition not found!");
      }
      g_is_dumping = false;
      g_req_dump = false;
    }

    // --- 1. Autocorrelation (Difference Function) ---
    // We want to find best lag L in range [MIN_PERIOD, MAX_PERIOD]

    int64_t autocorr_start = esp_timer_get_time();

    int64_t best_lag = 0;
    int64_t max_diff = 0;
    int64_t min_diff = INT64_MAX;

    const int start_lag = 200; // 20ms
    const int end_lag = 1000;  // 100ms
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

    // Process Slopes & Validate
    best_lag = 0;
    bool found_valid = false;

    // Need at least a few points
    if (count_lags > 3) {
      double max_slope = 0;
      static double slopes[MAX_LAGS]; // slope[i] is from i to i+1

      for (int i = 0; i < count_lags - 1; i++) {
        slopes[i] = (double)(errors[i + 1] - errors[i]);
        if (fabs(slopes[i]) > max_slope)
          max_slope = fabs(slopes[i]);
      }

      if (max_slope < 1.0)
        max_slope = 1.0;

      // Scan for Zero Crossings
      const int LAG_WINDOW = 1; // Window for d2 check

      for (int i = 0; i < count_lags - 2; i++) {
        // Normalize Slopes
        double norm_slope_curr = slopes[i] / max_slope;
        double norm_slope_next = slopes[i + 1] / max_slope;

        // Check for Negative -> Positive Slope (Valley)
        if (norm_slope_curr < 0 && norm_slope_next > 0) {
          int valley_idx = i + 1;

          // 1. Normalized Error Check < 0.5
          double norm_error = (double)(errors[valley_idx] - min_diff) /
                              (double)(max_diff - min_diff);

          if (norm_error < 0.5) {
            // 2. Curvature Check (Avg d2)
            // Range: [point - 2*LAG_WINDOW, point] which is valley_idx
            // We use slopes indices up to i (which is valley_idx - 1)
            // d2[k] corresponds to change in slope at k (slope[k+1] - slope[k])
            double d2_sum = 0;
            int count_d2 = 0;

            // We want to average derivatives of normalized slope ending at the
            // valley d2 calc uses slopes[k] and slopes[k+1]. To include the
            // transition OUT of the valley (slope[i] -> slope[i+1]), we look at
            // k=i. To look back 2*LAG_WINDOW steps...

            for (int k = i; k >= i - (2 * LAG_WINDOW); k--) {
              if (k < 0 || k >= count_lags - 1)
                continue;
              double d2 = (slopes[k + 1] - slopes[k]) / max_slope;
              d2_sum += d2;
              count_d2++;
            }

            if (count_d2 > 0) {
              double avg_d2 = d2_sum / count_d2;
              if (avg_d2 > 0.05) {
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
      // We know best_lag is from the coarse list, retrieve its error?
      // Just recompute local to be safe and simple

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

      // IQ Demodulation for Phase Tracking
      // Window: 4x Period
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

      // Ensure last_peak_timestamp is not essentially "unset" if calculation
      // fails? atan2 always returns value. Buffer always has data if we are
      // here (count check above).
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

      // Broadcast Config State if Idle (Throttle 0)
      if (g_control_input.throttle == 0) {
        g_config.type = APP_PACKET_TYPE_CONFIG_STATE;
        esp_now_send(s_broadcast_mac, (uint8_t *)&g_config, sizeof(g_config));
      }

      last_pkt_count = curr_pkt_count;
    }
  }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf || !info->len) {
    return;
  }
  if (g_config.rotation_source == APP_ROTATION_SOURCE_CSI) {
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
  // Extract RSSI
  if (g_config.rotation_source == APP_ROTATION_SOURCE_ESPNOW &&
      recv_info->rx_ctrl) {
    interpolate_rssi(&g_interpolated_rssi_buf, recv_info->rx_ctrl->timestamp,
                     recv_info->rx_ctrl->rssi);
    update_recv_stats(recv_info->rx_ctrl->rssi);
  }

  // Capture Target MAC
  if (!g_target_mac_set && recv_info->src_addr) {
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

  // Parse Packet
  if (data[0] == APP_PACKET_TYPE_CONTROL) {
    if (len >= sizeof(control_packet_t)) {
      const control_packet_t *pkt = (const control_packet_t *)data;
      g_control_input.throttle = pkt->throttle;
      g_control_input.vector_x = pkt->vector_x;
      g_control_input.vector_y = pkt->vector_y;
    }
  } else if (data[0] == APP_PACKET_TYPE_CMD_DUMP) {
    ESP_LOGI(TAG, "Received DUMP Command");
    g_req_dump = true;
  } else if (data[0] == APP_PACKET_TYPE_CONFIG_SET) {
    if (len >= sizeof(app_config_packet_t)) {
      const app_config_packet_t *pkt = (const app_config_packet_t *)data;
      ESP_LOGI(TAG, "Received SET CONFIG");

      bool reboot_needed = false;
      if (pkt->dshot_pin_a != g_config.dshot_pin_a ||
          pkt->dshot_pin_b != g_config.dshot_pin_b ||
          pkt->led_pin != g_config.led_pin) {
        reboot_needed = true;
      }

      // Update Global State
      g_config = *pkt;
      // Restore type just in case we need to send it back as STATE
      g_config.type = APP_PACKET_TYPE_CONFIG_STATE;

      save_config();

      if (reboot_needed) {
        ESP_LOGW(TAG, "Pin config changed. Rebooting...");
        esp_restart();
      }
    }
  }
}

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

  // Initialize LED Strip and Dim it (User Request)
  // Note: Only if LED strip component is available and valid pin
  led_strip_config_t strip_config = {
      .strip_gpio_num = g_config.led_pin,
      .max_leds = 1,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
  led_strip_set_pixel(g_led_strip, 0, 8, 8, 8); // ~3% brightness (very dim)
  led_strip_refresh(g_led_strip);

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
