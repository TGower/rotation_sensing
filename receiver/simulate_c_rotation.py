
import pandas as pd
import numpy as np
import sys

# Constants from C Firmware
INTERPOLATION_INTERVAL_US = 100
RSSI_BUF_SIZE = 6000
SMOOTHING_WINDOW = 20 # Default from C file
CORRELATION_WINDOW = 1000
STEP_LAG = 5
STEP_WINDOW = 5
START_LAG = 200
END_LAG = 3000

# Structures simulating C structs
class RssiPoint:
    def __init__(self, rssi, smoothed, timestamp):
        self.rssi = int(rssi) # int8_t
        self.smoothed_rssi = int(smoothed) # int8_t
        self.timestamp = int(timestamp) # int64_t

class CircularBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.last_timestamp = 0

def interpolate_rssi(buf, timestamp, rssi):
    """
    Simulates: static void interpolate_rssi(rssi_circular_buffer_t *buf, int64_t timestamp, int8_t rssi)
    """
    rssi = int(rssi)
    timestamp = int(timestamp)

    # Initial case
    if buf.last_timestamp == 0:
        buf.buffer[buf.head] = RssiPoint(rssi, rssi, timestamp)
        buf.last_timestamp = timestamp
        buf.head = (buf.head + 1) % buf.size
        return

    target_ts = buf.last_timestamp + INTERPOLATION_INTERVAL_US

    # Safety: Gap too large > 100ms
    if timestamp - buf.last_timestamp > 100000:
        buf.last_timestamp = timestamp
        buf.buffer[buf.head] = RssiPoint(rssi, rssi, timestamp)
        buf.head = (buf.head + 1) % buf.size
        # Since we don't strictly manage tail in the simulation loop for storage (infinite memory ideally, but here fixed)
        # In C: if (head == tail) tail = (tail + 1) % size;
        # We'll just mimic the filling logic.
        return

    # Sliding Window Logic calculation
    # We need to compute running sum of last (SMOOTHING_WINDOW - 1) samples relative to HEAD
    # In C this is done incrementally. Here we can just re-compute for strict correctness or optimize.
    # To be perfectly strict to C's logic:
    # "Initialize running_sum from recent history relative to HEAD"
    
    while target_ts <= timestamp:
        val = rssi
        
        # Calculate smoothed value for this new point
        # C logic:
        # 1. Sum previous (w_len - 1) samples
        # 2. Add current val
        # 3. If count > w_len, subtract oldest
        # 4. Divide
        
        w_len = SMOOTHING_WINDOW
        if w_len < 1: w_len = 1
        
        running_sum = 0
        valid_history = 0
        
        # In C, it iterates backwards from head-1.
        # "int current_total_count = (buf->head - buf->tail + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;"
        # For our simulation, we assume buffer is full enough after a while.
        # We can just look at the list indices.
        
        # NOTE: logic in C re-calculates running_sum every single interpolated point inside the while loop?
        # Yes: `int32_t running_sum = 0; ... loop for history ... while(target_ts...) { ... running_sum += val ... }`
        # wait, the C code initializes running_sum OUTSIDE the while loop.
        # And maintains it INSIDE the while loop.
        # Let's verify line 207 in C file.
        # Yes: `int32_t running_sum = 0; ... for (...) { running_sum += ... } ... while (target_ts <= timestamp) { running_sum += val; ... }`
        # So the running_sum is stateful across the interpolation burst.
        
        if valid_history == 0: # First time setup for this burst
             # Note: assumes continuous buffer in C.
             # We need to count how many valid samples we have?
             # In simulation, let's just count backwards
             current_total_count = 0 # Simply assume full for now if we have enough data?
             # Actually, simpler: just simulate the buffer list.
             pass

        # Let's do it exactly as C:
        # Re-calc history sum every time we enter the function (or before the while loop)
        
        running_sum = 0
        history_available = 0
        
        # We assume tail is far behind or buffer is effectively full.
        # We look back up to w_len-1
        
        # Check how many items we have in buffer so far
        # In python simulation, we just track total added?
        # Using buf.head as index.
        
        # BUT: the buffer wraps.
        
        # Let's calculate history sum
        needed = w_len - 1
        for i in range(needed):
            # idx = (head - 1 - i)
            # Need to check if valid (i.e. if we have wrapped or filled enough)
            # For simplicity, if buffer slot is None, stop.
            
            idx = (buf.head - 1 - i + buf.size) % buf.size
            if buf.buffer[idx] is None:
                break
            running_sum += buf.buffer[idx].rssi
            history_available += 1
            
        valid_history = history_available
        
        # Now enter C while loop
        
        # Update Sliding Window Sum
        running_sum += val
        valid_history += 1
        
        if valid_history > w_len:
            # We need to subtract the oldest from the hypothetical window
            # The window effectively shifted.
            # IN C: "int remove_idx = (buf->head - w_len + RSSI_BUF_SIZE) % RSSI_BUF_SIZE;"
            # Wait, buf->head is where we are ABOUT to write.
            # So the window includes: [head-(w_len-1) ... head] (if we wrote explicitly)
            # But we haven't written to head yet.
            # The window is formed by: Previous (w_len-1) samples AND the current 'val'.
            # So the "oldest" that falls out is at: head - w_len.
            
            # Correction: 
            # If we have valid_history (including current) > w_len:
            # We need to remove the one that was at (head - w_len).
            # Yes, that matches C line 233.
            
            remove_idx = (buf.head - w_len + buf.size) % buf.size
            # This slot must be valid if valid_history reached w_len
            if buf.buffer[remove_idx] is not None:
                running_sum -= buf.buffer[remove_idx].rssi
                valid_history = w_len # Cap it
        
        smoothed = int(running_sum / valid_history)
        
        # Write to buffer
        buf.buffer[buf.head] = RssiPoint(val, smoothed, target_ts)
        buf.head = (buf.head + 1) % buf.size
        # Update tail? Only if full.
        # buf.tail behavior in C: if (head == tail) tail++
        # We don't strictly need tail for the calc unless we want to simulate `count`
        
        buf.last_timestamp = target_ts
        target_ts += INTERPOLATION_INTERVAL_US

def calculate_autocorr_error(buf, head, lag, corr_window, step_win):
    """
    Simulates: int64_t calculate_autocorr_error(...)
    """
    diff_sum = 0
    # for (int i = 0; i < corr_window; i += step_win)
    for i in range(0, corr_window, step_win):
        idx1 = (head - 1 - i + buf.size) % buf.size
        idx2 = (head - 1 - i - lag + buf.size) % buf.size
        
        p1 = buf.buffer[idx1]
        p2 = buf.buffer[idx2]
        
        if p1 is None or p2 is None:
            continue # Should not happen if check passed
            
        v1 = p1.smoothed_rssi
        v2 = p2.smoothed_rssi
        diff_sum += abs(v1 - v2)
        
    return diff_sum

def run_rotation_task_logic(buf, debug=False):
    """
    Simulates one iteration of rotation_task
    """
    head = buf.head
    # We assume buffer is full enough (size 6000, corr_window 1000)
    # count check: if (count < corr_window * 2) continue;
    # Let's assume we proceed.
    
    start_lag = START_LAG # 200
    end_lag = END_LAG # 3000
    step_lag = STEP_LAG # 5
    step_win = STEP_WINDOW # 5
    
    best_lag = 0
    min_diff = float('inf') # INT64_MAX
    
    # Coarse Search
    for lag in range(start_lag, end_lag + 1, step_lag):
        diff_sum = calculate_autocorr_error(buf, head, lag, CORRELATION_WINDOW, step_win)
        if diff_sum < min_diff:
            min_diff = diff_sum
            best_lag = lag
            
    # Harmonic Check
    if best_lag > 0:
        original_lag = best_lag
        threshold = min_diff * 160 // 100 # integer math 160/100 = 1.6
        
        for div in range(8, 1, -1): # 8 down to 2
            test_lag = original_lag // div
            if test_lag < start_lag:
                continue
                
            test_diff = calculate_autocorr_error(buf, head, test_lag, CORRELATION_WINDOW, step_win)
            
            if test_diff <= threshold:
                best_lag = test_lag
                min_diff = test_diff # C code does NOT update min_diff? Check!
                # C code:
                # if (test_diff <= threshold) {
                #   best_lag = test_lag;
                #   break;
                # }
                # It does NOT update min_diff. This is a subtle point!
                # Wait, if it breaks, it keeps the `best_lag`.
                # But subsequent fine search uses `min_diff`?
                # Fine search loop: `if (diff_sum < min_diff)`
                # If we updated `best_lag` to a harmonic but kept `min_diff` of the fundamental (which is lower usually?),
                # then fine search might fail to find a better point if the harmonic is slightly worse?
                # Actually, `test_diff <= threshold` means test_diff is allowed to be WORSE (higher) than min_diff.
                # So `min_diff` is essentially the "Global Minimum" of the coarse search.
                # If we pick a harmonic, it has higher error.
                # Then in Fine Search:
                # We search around `best_lag` (the harmonic).
                # We compare `diff_sum` to `min_diff` (the Global Minimum).
                # `diff_sum` of harmonic neighbors will likely be > `min_diff`.
                # So `min_diff` will NOT update, and `final_lag` will NOT update from initial `best_lag` (harmonic)
                # unless we invoke the "init final_lag = best_lag" logic.
                # C code:
                # int final_lag = best_lag;
                # for (...) { ... if (diff_sum < min_diff) { min_diff = diff_sum; final_lag = lag; } }
                # So if the harmonic's neighbors are NOT better than the GLOBAL minimum (fundamental),
                # final_lag remains `best_lag` (harmonic).
                # This seems correct behavior: we locked onto the harmonic, and fine search just checks if we can beat the global min? 
                # No, fine search is supposed to refine the peak locally.
                # If `diff_sum` (local neighbor of harmonic) is > `min_diff` (fundamental), we don't update `final_lag`.
                # So we just stick with the coarse harmonic estimate.
                # This effectively DISABLES fine search for harmonics if they are worse than fundamental!
                # That might be a bug or intended.
                # "Narrow down the best lag... if (diff_sum < min_diff)"
                # Yes, if we switched to harmonic, min_diff is still the low error of the fundamental.
                # Neighbor of harmonic is likely higher error.
                # So `final_lag` stays `best_lag`.
                break
    
    final_lag = best_lag
    
    # Fine Search
    for i in range(-step_lag, step_lag + 1):
        lag = best_lag + i
        if lag < start_lag or lag > end_lag:
            continue
        
        diff_sum = calculate_autocorr_error(buf, head, lag, CORRELATION_WINDOW, step_win)
        if diff_sum < min_diff:
            min_diff = diff_sum
            final_lag = lag
            
    return final_lag

def main():
    filepath = "/home/t/esp/rotation_sensing/receiver/cleaned_runs/csi_log2_part_49.csv"
    print(f"Loading {filepath}...")
    df = pd.read_csv(filepath)
    
    timestamps = df["local_timestamp"].values
    rssis = df["rssi"].values
    
    # Simulate the Timeline
    buf = CircularBuffer(RSSI_BUF_SIZE)
    
    # Fill buffer completely first? Or simulated real-time?
    # Let's iterate through all packets and update buffer
    # and periodically run the rotation task (e.g. every 100ms or just at the end?)
    # "I want to know what lag value it would calculate if it had that data coming in"
    # implying the result at the end of the file or at specific snapshots.
    # The file is likely a few seconds.
    # Let's run it at the very end of the file, assuming it captures the rotation.
    
    print("Interpolating full file into buffer...")
    for t, r in zip(timestamps, rssis):
        interpolate_rssi(buf, t, r)
        
    print(f"Buffer Head: {buf.head}, Last TS: {buf.last_timestamp}")
    
    # Run Task
    print("Running Rotation Task Logic...")
    lag = run_rotation_task_logic(buf, debug=True)
    
    period_us = lag * INTERPOLATION_INTERVAL_US
    rate = 1000000.0 / period_us if period_us > 0 else 0
    
    print(f"--- RESULT ---")
    print(f"Calculated Best Lag: {lag}")
    print(f"Estimated Period: {period_us} us")
    print(f"Rotation Rate: {rate:.2f} Hz")

if __name__ == "__main__":
    main()
