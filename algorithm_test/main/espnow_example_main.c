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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LEDC_IO 48
#define ESPNOW_MAXDELAY 512

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz
#define DSHOT_ESC_GPIO_NUM_A 13
#define DSHOT_ESC_GPIO_NUM_B 14

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

typedef struct {
  int8_t rssi[RSSI_BUF_SIZE] __attribute__((aligned(16)));
  int8_t smoothed_rssi[RSSI_BUF_SIZE] __attribute__((aligned(16)));
  int64_t timestamp[RSSI_BUF_SIZE] __attribute__((aligned(16)));
  int head;
  int tail;
  int64_t last_timestamp;
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

static rssi_circular_buffer_t g_csi_rssi_buf = {0};
static rssi_circular_buffer_t g_espnow_rssi_buf = {0};
static control_input_t g_control_input = {0};
static rotation_state_t g_rotation_state = {0};

static app_config_packet_t g_config = {.type = APP_PACKET_TYPE_CONFIG_STATE,
                                       .dshot_pin_a = DSHOT_ESC_GPIO_NUM_A,
                                       .dshot_pin_b = DSHOT_ESC_GPIO_NUM_B,
                                       .led_pin = LEDC_IO,
                                       .rotation_source = 0, // CSI
                                       .step_lag = 5,
                                       .step_window = 5,
                                       .smoothing_window = 20,
                                       .throttle_multiplier = 1.0f,
                                       .translation_multiplier = 1.0f,
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

// DShot Handles
static rmt_channel_handle_t esc_chan_a = NULL;
static rmt_channel_handle_t esc_chan_b = NULL;
static rmt_encoder_handle_t dshot_encoder_a = NULL;
static rmt_encoder_handle_t dshot_encoder_b = NULL;
static led_strip_handle_t g_led_strip = NULL;

// Helper: Interpolate and Add to Circular Buffer
static void interpolate_rssi(rssi_circular_buffer_t *buf, int64_t timestamp,
                             int8_t rssi) {
  // If buffer is empty, just add the first point
  if (buf->last_timestamp == 0) {
    buf->rssi[buf->head] = rssi;
    buf->smoothed_rssi[buf->head] = rssi; // No history yet
    buf->timestamp[buf->head] = timestamp;
    buf->last_timestamp = timestamp;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    // Tail stays 0 until full? Or just 0.
    return;
  }

  int64_t target_ts = buf->last_timestamp + INTERPOLATION_INTERVAL_US;

  // Safety: If gap is too large (> 100ms), reset
  if (timestamp - buf->last_timestamp > 100000) {
    buf->last_timestamp = timestamp;
    buf->rssi[buf->head] = rssi;
    buf->smoothed_rssi[buf->head] = rssi; // Reset history
    buf->timestamp[buf->head] = timestamp;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    if (buf->head == buf->tail) {
      buf->tail = (buf->tail + 1) % RSSI_BUF_SIZE;
    }
    return;
  }

  // Nearest Neighbor Interpolation for all uniform points between last_ts and
  // current ts Setup sliding window for smoothing from existing buffer state
  uint16_t w_len = g_config.smoothing_window;
  if (w_len < 1)
    w_len = 1;

  int32_t running_sum = 0;
  int valid_history = 0;

  // Initialize running_sum from recent history relative to HEAD
  // We need (w_len - 1) samples.
  int current_total_count =
      (buf->head - buf->tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
  int history_available =
      (current_total_count > (w_len - 1)) ? (w_len - 1) : current_total_count;

  for (int i = 0; i < history_available; i++) {
    int idx = (buf->head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    running_sum += buf->rssi[idx];
  }
  valid_history = history_available;

  while (target_ts <= timestamp) {
    // Nearest neighbor
    int8_t val = rssi;

    // Update Sliding Window Sum
    running_sum += val;
    valid_history++;

    if (valid_history > w_len) {
      // Subtract oldest
      int remove_idx = (buf->head - w_len + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
      running_sum -= buf->rssi[remove_idx];
      valid_history = w_len;
    }

    int8_t smoothed = (int8_t)(running_sum / valid_history);

    buf->rssi[buf->head] = val;
    buf->smoothed_rssi[buf->head] = smoothed;
    buf->timestamp[buf->head] = target_ts;
    buf->head = (buf->head + 1) % RSSI_BUF_SIZE;
    if (buf->head == buf->tail) {
      buf->tail = (buf->tail + 1) % RSSI_BUF_SIZE;
    }

    buf->last_timestamp = target_ts;
    target_ts += INTERPOLATION_INTERVAL_US;
  }
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

    ESP_LOGI(TAG, "Left DShot: %d, Right DShot: %d", leftDShot, rightDShot);

    // --- Update LED ---
    // Green in 45 deg arc opposite peak.
    if (phase > HEADING_START && phase < HEADING_END) {
      led_strip_set_pixel(g_led_strip, 0, 0, 20, 0); // Green
    } else {
      led_strip_set_pixel(g_led_strip, 0, 20, 0, 0); // Red
    }
    led_strip_refresh(g_led_strip);
  }
}

// Rotation Estimation Task
// Helper for Autocorrelation Error Calculation
static int64_t calculate_autocorr_error(rssi_circular_buffer_t *buf, int head,
                                        int lag, int corr_window,
                                        int step_win) {
  int64_t diff_sum = 0;
  int i = 0;
  // Process 4 samples at a time for performance
  for (; i + 3 * step_win < corr_window; i += 4 * step_win) {
    int idx1_0 = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx2_0 = (head - 1 - i - lag + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx1_1 = (idx1_0 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx2_1 = (idx2_0 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx1_2 = (idx1_1 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx2_2 = (idx2_1 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx1_3 = (idx1_2 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx2_3 = (idx2_2 - step_win + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;

    diff_sum += abs(buf->smoothed_rssi[idx1_0] - buf->smoothed_rssi[idx2_0]);
    diff_sum += abs(buf->smoothed_rssi[idx1_1] - buf->smoothed_rssi[idx2_1]);
    diff_sum += abs(buf->smoothed_rssi[idx1_2] - buf->smoothed_rssi[idx2_2]);
    diff_sum += abs(buf->smoothed_rssi[idx1_3] - buf->smoothed_rssi[idx2_3]);
  }

  // Cleanup remainder
  for (; i < corr_window; i += step_win) {
    int idx1 = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    int idx2 = (head - 1 - i - lag + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    diff_sum += abs(buf->smoothed_rssi[idx1] - buf->smoothed_rssi[idx2]);
  }
  return diff_sum;
}

// Rotation Estimation Task
static void rotation_task(void *pvParameter) {

  while (1) {
    vTaskDelay(1);
    rssi_circular_buffer_t *active_buf =
        (g_config.rotation_source == 0) ? &g_csi_rssi_buf : &g_espnow_rssi_buf;

    int head = active_buf->head;
    int count = (head - active_buf->tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
    uint16_t corr_window = g_config.correlation_window;
    if (corr_window == 0)
      corr_window = 1000; // Safety default

    if (count < corr_window * 2)
      continue; // Need enough data

    // --- 1. Autocorrelation (Difference Function) ---
    // We want to find best lag L in range [MIN_PERIOD, MAX_PERIOD]

    int64_t autocorr_start = esp_timer_get_time();

    int32_t best_lag = 0;
    int64_t min_diff = INT64_MAX;

    int start_lag = 200; // 20ms
    int end_lag = 3000;  // 300ms

    // Subsample for speed
    int step_lag = g_config.step_lag;
    int step_win = g_config.step_window;

    // Coarse Search
    for (int lag = start_lag; lag <= end_lag; lag += step_lag) {
      int64_t diff_sum = calculate_autocorr_error(active_buf, head, lag,
                                                  corr_window, step_win);
      if (diff_sum < min_diff) {
        min_diff = diff_sum;
        best_lag = lag;
      }
    }

    // Check for integer multiples (Harmonics)
    if (best_lag > 0) {
      int original_lag = best_lag;
      int64_t threshold = min_diff * 160 / 100; // Within 60% of min error

      for (int div = 8; div >= 2; div--) {
        int test_lag = original_lag / div;
        if (test_lag < start_lag)
          continue;

        int64_t test_diff = calculate_autocorr_error(active_buf, head, test_lag,
                                                     corr_window, step_win);

        if (test_diff <= threshold) {
          best_lag = test_lag;
          break;
        }
      }
    }

    int final_lag = best_lag;

    // Narrow down the best lag, checking +- step_lag
    for (int i = -step_lag; i <= step_lag; i++) {
      int lag = best_lag + i;
      if (lag < start_lag || lag > end_lag)
        continue;
      int64_t diff_sum = calculate_autocorr_error(active_buf, head, lag,
                                                  corr_window, step_win);
      if (diff_sum < min_diff) {
        min_diff = diff_sum;
        final_lag = lag;
      }
    }

    if (final_lag > 0) {
      g_rotation_state.estimated_period_us =
          final_lag * INTERPOLATION_INTERVAL_US;
      g_rotation_state.rotation_rate =
          1000000.0f / g_rotation_state.estimated_period_us;

      // Find Phase Peak in the second most recent period
      int peak_idx = -1;
      int8_t max_rssi = -128;
      for (int i = final_lag; i < final_lag * 2; i++) {
        int idx = (head - 1 - i + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;
        if (active_buf->smoothed_rssi[idx] > max_rssi) {
          max_rssi = active_buf->smoothed_rssi[idx];
          peak_idx = idx;
        }
      }
      if (peak_idx >= 0) {
        g_rotation_state.last_peak_timestamp =
            active_buf->timestamp[(peak_idx + final_lag) % RSSI_BUF_SIZE];
      }
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
      buf_stats_t csi_stats, espnow_stats;
      calculate_stats(&g_csi_rssi_buf, &csi_stats);
      calculate_stats(&g_espnow_rssi_buf, &espnow_stats);

      ESP_LOGI(TAG,
               "Stats: %" PRIu32 " pkts/sec | Last RSSI: %d | Throttle: %d",
               diff, g_last_rssi, g_control_input.throttle);
      ESP_LOGI(TAG, "Vector: %f, %f", g_control_input.vector_x,
               g_control_input.vector_y);

      // Send Stats Packet back to Sender
      stats_packet_t stats_pkt = {.type = APP_PACKET_TYPE_STATS,
                                  .csi_mean = csi_stats.mean,
                                  .csi_var = csi_stats.variance,
                                  .espnow_mean = espnow_stats.mean,
                                  .espnow_var = espnow_stats.variance,
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

  interpolate_rssi(&g_csi_rssi_buf, esp_timer_get_time(), info->rx_ctrl.rssi);
  update_recv_stats(info->rx_ctrl.rssi);
}

// Send Callback
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  g_send_status = (status == ESP_NOW_SEND_SUCCESS);
  xSemaphoreGiveFromISR(g_send_cb_sem, NULL);
}

// Task to flush the buffer
// Removed flush_task

// ESP-NOW Receive Callback - PASSIVE, just updates timestamp
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                                   const uint8_t *data, int len) {
  // Extract RSSI
  if (recv_info->rx_ctrl) {
    interpolate_rssi(&g_espnow_rssi_buf, esp_timer_get_time(),
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

#include "test_data.h"

// Assembly function declaration
extern int32_t calculate_sad_vector(const int8_t *A, const int8_t *B, int len,
                                    const int8_t *ones);

// Aligned Vector of Ones (16 bytes)
static const int8_t aligned_ones[16] __attribute__((aligned(16))) = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// SIMD Autocorrelation Wrapper
static int64_t calculate_autocorr_simd(rssi_circular_buffer_t *buf, int head,
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
    while (block_len > 0 && ((uintptr_t)&buf->smoothed_rssi[cur_idx] & 0xF)) {
      total_diff +=
          abs(buf->smoothed_rssi[cur_idx] - buf->smoothed_rssi[idx_B]);

      cur_idx = (cur_idx + 1) % RSSI_BUF_SIZE;
      idx_B = (idx_B + 1) % RSSI_BUF_SIZE;
      block_len--;
      samples_left--;
    }

    // Now A is aligned (or block_len is 0)
    int simd_len = (block_len / 16) * 16;
    if (simd_len > 0) {
      total_diff += calculate_sad_vector(&buf->smoothed_rssi[cur_idx],
                                         &buf->smoothed_rssi[idx_B], simd_len,
                                         aligned_ones);

      cur_idx = (cur_idx + simd_len) % RSSI_BUF_SIZE;
      idx_B = (idx_B + simd_len) % RSSI_BUF_SIZE;
      block_len -= simd_len;
      samples_left -= simd_len;
    }

    // Handle remainder
    while (block_len > 0) {
      total_diff +=
          abs(buf->smoothed_rssi[cur_idx] - buf->smoothed_rssi[idx_B]);
      cur_idx = (cur_idx + 1) % RSSI_BUF_SIZE;
      idx_B = (idx_B + 1) % RSSI_BUF_SIZE;
      block_len--;
      samples_left--;
    }
  }

  return total_diff;
}

// Instrument Autocorrelation
static int64_t
calculate_autocorr_error_instrumented(rssi_circular_buffer_t *buf, int head,
                                      int lag, int corr_window, int step_win) {
  // int64_t start = esp_timer_get_time();
  // // Use Original
  // int64_t val = calculate_autocorr_error(buf, head, lag, corr_window,
  // step_win); int64_t end = esp_timer_get_time();

  int64_t start_opt = esp_timer_get_time();
  // Use SIMD (Processes ALL samples, so results will differ from step_win=5)
  int64_t val_opt = calculate_autocorr_simd(buf, head, lag, corr_window);
  int64_t end_opt = esp_timer_get_time();

  // Print: Lag, Error, TimeUS
  printf("BENCH: %d, %" PRId64 ", %" PRId64 "\n", lag, val_opt,
         (end_opt - start_opt));
  return val_opt;
}

// Benchmark Task
static void benchmark_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Benchmark Task with %d samples...",
           (int)test_data_len);

  // Config for benchmark
  g_config.rotation_source = 0; // Use CSI buffer
  g_config.correlation_window = 1000;
  g_config.step_lag = 5;
  g_config.step_window = 5;
  g_config.smoothing_window = 20;

  int corr_window = g_config.correlation_window;
  int step_win = g_config.step_window;

  vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for serial

  while (1) {
    // Reset Buffer for each run to be clean
    memset(&g_csi_rssi_buf, 0, sizeof(rssi_circular_buffer_t));

    ESP_LOGI(TAG, "--- Starting New Benchmark Run ---");
    ESP_LOGI(TAG, "Format: BENCH: Lag, Error, TimeUS");

    int samples_processed = 0;

    for (size_t i = 0; i < test_data_len; i++) {
      // Simulate packet arrival
      int64_t ts = test_timestamps[i];
      int8_t rssi = test_rssi[i];

      // Use interpolate_rssi to fill buffer
      interpolate_rssi(&g_csi_rssi_buf, ts, rssi);

      int head = g_csi_rssi_buf.head;
      int count = (head - g_csi_rssi_buf.tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;

      // Only run if we have enough data
      if (count >= corr_window * 2) {
        samples_processed++;
        ESP_LOGI(TAG, "Sample %d | TS: %" PRId64 " | RSSI: %d", (int)i, ts,
                 rssi);

        // --- 1. Autocorrelation (Difference Function) ---
        int32_t best_lag = 0;
        int64_t min_diff = INT64_MAX;

        int start_lag = 200; // 20ms
        int end_lag = 3000;  // 300ms

        // Subsample for speed
        int step_lag = g_config.step_lag;
        // int step_win = g_config.step_window; // Already defined above

        // Coarse Search
        for (int lag = start_lag; lag <= end_lag; lag += step_lag) {
          int64_t diff_sum = calculate_autocorr_error_instrumented(
              &g_csi_rssi_buf, head, lag, corr_window, step_win);
          if (diff_sum < min_diff) {
            min_diff = diff_sum;
            best_lag = lag;
          }
        }

        // Check for integer multiples (Harmonics)
        if (best_lag > 0) {
          int original_lag = best_lag;
          int64_t threshold = min_diff * 160 / 100; // Within 60% of min error

          for (int div = 8; div >= 2; div--) {
            int test_lag = original_lag / div;
            if (test_lag < start_lag)
              continue;

            int64_t test_diff = calculate_autocorr_error_instrumented(
                &g_csi_rssi_buf, head, test_lag, corr_window, step_win);

            if (test_diff <= threshold) {
              best_lag = test_lag;
              break;
            }
          }
        }

        int final_lag = best_lag;

        // Narrow down the best lag, checking +- step_lag
        for (int k = -step_lag; k <= step_lag; k++) {
          int lag = best_lag + k;
          if (lag < start_lag || lag > end_lag)
            continue;
          int64_t diff_sum = calculate_autocorr_error_instrumented(
              &g_csi_rssi_buf, head, lag, corr_window, step_win);
          if (diff_sum < min_diff) {
            min_diff = diff_sum;
            final_lag = lag;
          }
        }

        // Print final result summary for this sample
        printf("RESULT: %" PRId64 ", %d, %d, %" PRId64 "\n", ts, rssi,
               final_lag, min_diff);

        if (samples_processed >= 5) {
          ESP_LOGI(TAG, "Limit of 5 samples reached. Finishing run.");
          break;
        }
      }

      // Small delay to allow serial to flush, otherwise buffer overflows
      // With so much printing per sample, we really need to slow down loop or
      // increase buffer Or just benchmark a few samples? User requested
      // benchmark... lets wait a bit more
      if (i % 10 == 0)
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "Run Complete. Restarting in 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting Benchmark Application...");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  load_config();

  // Create Benchmark Task
  xTaskCreatePinnedToCore(benchmark_task, "benchmark_task", 4096, NULL, 5, NULL,
                          1);
}
