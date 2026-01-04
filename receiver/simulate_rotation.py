import csv
import math
import matplotlib.pyplot as plt

# Configuration
CSV_FILE = 'dump_slot03.csv'
INTERPOLATION_INTERVAL_US = 100
START_LAG = 200
END_LAG = 1000
STEP_LAG = 5
CORR_WINDOW = 1000
LAG_WINDOW = 1

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
    # data is a list. head is the index "after" the last element (len(data)).
    # We want to compare range [head-window, head) with [head-window-lag, head-lag)
    
    # Python slicing: [start:end]
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

def calculate_iq_phase(data, head, period_samples):
    # Perform I/Q Demodulation (Single Bin DFT) over 4 periods looking back from head
    # Window size = 4 * period_samples
    # Frequency = 1 / period_samples
    
    window_samples = period_samples * 4
    
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

# State Machine
class RotationState:
    def __init__(self):
        self.rotation_rate = 0.5 # Default 0.5 Hz
        self.estimated_period_us = 2000000 # 2 seconds
        # self.last_peak_timestamp = 0 # Removed
        self.valid_lock = False

def run_rotation_task(data, timestamps, head, state):
    # Check for enough data
    count = head
    if count < CORR_WINDOW * 2:
        return

    # 1. Coarse Search
    min_diff = float('inf')
    max_diff = float('-inf')
    
    lags = []
    errors = []
    
    # Simple Python loop (could be optimized but sufficient for 10000 ops)
    # We only check specific steps
    curr_lag_list = range(START_LAG, END_LAG + 1, STEP_LAG)
    
    for lag in curr_lag_list:
        err = calculate_autocorr_error(data, head, lag, CORR_WINDOW)
        lags.append(lag)
        errors.append(err)
        if err < min_diff: min_diff = err
        if err > max_diff: max_diff = err

    # 2. Process Slopes
    # Need at least a few points
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
    
    # Scan for zero crossings
    for i in range(len(slopes) - 1):
        norm_curr = slopes[i] / max_slope
        norm_next = slopes[i+1] / max_slope
        
        if norm_curr < 0 and norm_next > 0:
            valley_idx = i + 1
            
            # 1. Norm Error Check
            norm_error = (errors[valley_idx] - min_diff) / (max_diff - min_diff)
            if norm_error < 0.5:
                # 2. Curvature (d2)
                d2_sum = 0
                count_d2 = 0
                # Look back
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
                    break # First valid
                    
    final_lag = best_lag
    
    if not found_valid:
        # Loss of lock logic?
        # The C code sets defaults if NO valid found throughout sweep
        # But we only reset if we fail. 
        # C code: if (!found_valid) { ... defaults ... }
        state.rotation_rate = 0.5
        state.estimated_period_us = 2000000
        state.valid_lock = False
        final_lag = 0
    else:
        # Fine search
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
        
        # Note: We no longer look for Phase Peak here. 
        # Phase is calculated continuously using IQ.

def main():
    import sys
    import os

    if len(sys.argv) > 1:
        csv_filename = sys.argv[1]
    else:
        csv_filename = 'dump_slot00.csv' # Default to 00 if not specified
        
    print(f"Loading {csv_filename}...")
    try:
        timestamps, smoothed_rssi = load_data(csv_filename)
    except FileNotFoundError:
        print(f"Error: {csv_filename} not found.")
        return
        
    print(f"Loaded {len(smoothed_rssi)} samples.")
    
    state = RotationState()
    
    # Output arrays
    out_timestamps = []
    out_rssi = []
    out_rate = []
    out_phase = []
    out_lock = []
    
    # Simulation Parameters
    UPDATE_INTERVAL = 100 # Run frequency task every 100 samples (10ms)
    
    print("Running Simulation Loop...")
    
    for i in range(len(timestamps)):
        current_ts = timestamps[i]
        
        # Run Task
        if i >= 2000 and i % UPDATE_INTERVAL == 0:
             run_rotation_task(smoothed_rssi, timestamps, i, state)
             
        # Calculate Phase
        phase = 0.0
        if state.valid_lock:
             # Calculate IQ Phase over one estimated period
             period_samples = int(state.estimated_period_us / INTERPOLATION_INTERVAL_US)
             if i >= period_samples:
                 phase = calculate_iq_phase(smoothed_rssi, i, period_samples)
        
        # Store
        out_timestamps.append(current_ts)
        out_rssi.append(smoothed_rssi[i])
        out_rate.append(state.rotation_rate)
        out_phase.append(phase)
        out_lock.append(1 if state.valid_lock else 0)

    print("Simulation Complete. Plotting...")
    
    # Plotting
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 1. RSSI
    ax1.plot(out_timestamps, out_rssi, 'b-', label='RSSI')
    ax1.set_ylabel('RSSI (dBm)')
    ax1.set_title(f'Tracked Rotation Parameters ({os.path.basename(csv_filename)}) - IQ Demodulation')
    ax1.grid(True)
    
    # 2. Rate
    ax2.plot(out_timestamps, out_rate, 'g-', label='Estimated Rate (Hz)')
    ax2.set_ylabel('Rate (Hz)')
    ax2.grid(True)
    
    # 3. Phase
    ax3.plot(out_timestamps, out_phase, 'r.', markersize=1, label='Phase (rad)')
    ax3.set_ylabel('Phase (0-2pi)')
    ax3.set_ylim(0, 2*math.pi)
    ax3.grid(True)
    
    # Overlay phase 0 crossings
    peak_pred_timestamps = []
    for k in range(1, len(out_phase)):
        if out_phase[k] < out_phase[k-1] - math.pi:
            # Wrap around
            peak_pred_timestamps.append(out_timestamps[k])
            
    # Visualize predicted peaks on RSSI graph
    for ts in peak_pred_timestamps:
        ax1.axvline(x=ts, color='orange', alpha=0.3, linestyle='--')
        
    ax3.set_xlabel('Timestamp (us)')
    plt.tight_layout()
    
    # Derive output filename
    base_name = os.path.splitext(os.path.basename(csv_filename))[0]
    out_png = f"simulation_tracking_{base_name}_iq.png"
    
    plt.savefig(out_png)
    print(f"Saved {out_png}")


if __name__ == "__main__":
    main()
