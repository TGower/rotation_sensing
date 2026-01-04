import csv
import math
import matplotlib.pyplot as plt
import sys
import os

CSV_FILE = 'dump_slot02.csv'
START_LAG = 200
END_LAG = 1000
STEP_LAG = 5
CORR_WINDOW = 1000
LAG_WINDOW = 1
INTERPOLATION_INTERVAL_US = 100

def load_data(filename):
    timestamps = []
    smoothed_rssi = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(int(row['Timestamp_US']))
            smoothed_rssi.append(int(row['RSSI']))
            
    # Sort by timestamp to handle circular buffer wrapping
    combined = sorted(zip(timestamps, smoothed_rssi))
    timestamps = [x[0] for x in combined]
    smoothed_rssi = [x[1] for x in combined]
    
    return timestamps, smoothed_rssi

def calculate_autocorr_error(data, head, lag, window):
    start_A = head - window
    end_A = head
    start_B = head - window - lag
    end_B = head - lag
    
    if start_B < 0:
        raise ValueError("Not enough data for lag calculation")
        
    vec_A = data[start_A : end_A]
    vec_B = data[start_B : end_B]
    
    diff_sum = sum(abs(a - b) for a, b in zip(vec_A, vec_B))
    return diff_sum

def calculate_iq_phase_with_multiplier(data, head, period_samples, multiplier):
    # Perform I/Q Demodulation (Single Bin DFT) over multiplier * periods
    window_samples = int(period_samples * multiplier)
    
    if head < window_samples:
        return 0.0
        
    start_idx = head - window_samples
    window_data = data[start_idx : head]
    
    I = 0.0
    Q = 0.0
    
    # Angular frequency for the target period
    omega = 2.0 * math.pi / period_samples
    
    for k in range(window_samples):
        # delta ranges from -window_samples to -1 relative to head
        delta = k - window_samples
        val = window_data[k]
        
        angle = omega * delta
        I += val * math.cos(angle)
        Q += val * math.sin(angle)
        
    if I == 0 and Q == 0:
        return 0.0
        
    # Phase phi such that Signal ~ cos(omega*t + phi)
    phi = math.atan2(-Q, I)
    
    # Normalize to 0..2pi
    if phi < 0:
        phi += 2.0 * math.pi
        
    return phi

class RotationState:
    def __init__(self):
        self.rotation_rate = 0.5 
        self.estimated_period_us = 2000000 
        self.valid_lock = False

def run_rotation_task(data, timestamps, head, state):
    count = head
    if count < CORR_WINDOW * 2:
        return

    min_diff = float('inf')
    max_diff = float('-inf')
    lags = []
    errors = []
    
    curr_lag_list = range(START_LAG, END_LAG + 1, STEP_LAG)
    
    for lag in curr_lag_list:
        err = calculate_autocorr_error(data, head, lag, CORR_WINDOW)
        lags.append(lag)
        errors.append(err)
        if err < min_diff: min_diff = err
        if err > max_diff: max_diff = err

    if len(lags) < 4: return

    slopes = []
    max_slope = 0.0
    for i in range(len(errors) - 1):
        s = errors[i+1] - errors[i]
        slopes.append(s)
        if abs(s) > max_slope:
            max_slope = abs(s)
            
    if max_slope < 1.0: max_slope = 1.0
    
    best_lag = 0
    found_valid = False
    
    for i in range(len(slopes) - 1):
        norm_curr = slopes[i] / max_slope
        norm_next = slopes[i+1] / max_slope
        
        if norm_curr < 0 and norm_next > 0:
            valley_idx = i + 1
            norm_error = (errors[valley_idx] - min_diff) / (max_diff - min_diff)
            if norm_error < 0.5:
                d2_sum = 0
                count_d2 = 0
                for k in range(i, i - (2 * LAG_WINDOW) - 1, -1):
                    if k < 0: continue
                    d2 = (slopes[k+1] - slopes[k]) / max_slope
                    d2_sum += d2
                    count_d2 += 1
                
                avg_d2 = 0
                if count_d2 > 0: avg_d2 = d2_sum / count_d2
                
                if avg_d2 > 0.05:
                    best_lag = lags[valley_idx]
                    found_valid = True
                    break 
                    
    final_lag = best_lag
    
    if not found_valid:
        state.rotation_rate = 0.5
        state.estimated_period_us = 2000000
        state.valid_lock = False
    else: # Found valid
        fine_min = float('inf')
        real_final_lag = final_lag
        for lag in range(best_lag - STEP_LAG, best_lag + STEP_LAG + 1):
            if lag < START_LAG or lag > END_LAG: continue
            err = calculate_autocorr_error(data, head, lag, CORR_WINDOW)
            if err < fine_min:
                fine_min = err
                real_final_lag = lag
        
        final_lag = real_final_lag
        state.valid_lock = True
        state.estimated_period_us = final_lag * INTERPOLATION_INTERVAL_US
        state.rotation_rate = 1000000.0 / state.estimated_period_us


def main():
    print(f"Loading {CSV_FILE}...")
    try:
        timestamps, smoothed_rssi = load_data(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: {CSV_FILE} not found.")
        return
        
    print(f"Loaded {len(smoothed_rssi)} samples.")
    
    # Prepare data storage
    multipliers = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
    results = {m: {'phase': [], 'timestamps': []} for m in multipliers}
    
    state = RotationState()
    UPDATE_INTERVAL = 100 
    
    print("Running Simulation Loop...")
    
    # Pre-fill None/0 for startup
    # We'll just append and plot valid range? 
    # Or just start plotting from t=0 but with 0 values.
    
    for i in range(len(timestamps)):
        current_ts = timestamps[i]
        
        # Run Rate Estimation
        if i >= 2000 and i % UPDATE_INTERVAL == 0:
             run_rotation_task(smoothed_rssi, timestamps, i, state)
             
        # Calculate Phase for each multiplier
        for m in multipliers:
            phase = 0.0
            if state.valid_lock:
                 period_samples = int(state.estimated_period_us / INTERPOLATION_INTERVAL_US)
                 if i >= int(period_samples * m):
                     phase = calculate_iq_phase_with_multiplier(smoothed_rssi, i, period_samples, m)
            
            results[m]['phase'].append(phase)
            results[m]['timestamps'].append(current_ts)

    print("Simulation Complete. Plotting...")
    
    # 3x3 Grid
    fig, axes = plt.subplots(3, 3, figsize=(18, 12), sharex=True, sharey=True)
    axes = axes.flatten()
    
    for idx, m in enumerate(multipliers):
        ax = axes[idx]
        ts = results[m]['timestamps']
        ph = results[m]['phase']
        
        ax.plot(ts, ph, 'r.', markersize=1)
        ax.set_title(f"Window: {m}x Period")
        ax.set_ylim(0, 2 * math.pi)
        ax.grid(True)
        
        if idx >= 6: # Bottom row
            ax.set_xlabel('Timestamp (us)')
        if idx % 3 == 0: # Left column
            ax.set_ylabel('Phase (rad)')
            
    plt.suptitle(f"Phase Tracking vs Window Size ({CSV_FILE})")
    plt.tight_layout()
    plt.savefig('simulation_window_sweep.png')
    print("Saved simulation_window_sweep.png")

if __name__ == "__main__":
    main()
