import csv
import math
import matplotlib.pyplot as plt
import sys
import os
import argparse
import numpy as np
import finufft

# Configuration
INTERPOLATION_INTERVAL_US = 100
START_LAG = 200
END_LAG = 1000
STEP_LAG = 5
CORR_WINDOW = 1000
LAG_WINDOW = 1

class RotationState:
    def __init__(self):
        self.rotation_rate = 0.5 # Default 0.5 Hz
        self.estimated_period_us = 2000000 # 2 seconds
        self.valid_lock = False

def load_raw_data(filename):
    _raw_timestamps = []
    _raw_rssi = []
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts = int(row['Timestamp_US'])
                rssi = int(row['RSSI'])
                if ts > 0:
                   _raw_timestamps.append(ts)
                   _raw_rssi.append(rssi)
            except ValueError:
                continue

    if not _raw_timestamps:
        return [], []
            
    # Sort by timestamp to handle circular buffer wrapping
    combined = sorted(zip(_raw_timestamps, _raw_rssi))
    _raw_timestamps = [x[0] for x in combined]
    _raw_rssi = [x[1] for x in combined]
    
    return _raw_timestamps, _raw_rssi

def calculate_autocorr_error(data, head, lag, window):
    # data is a list. head is the index "after" the last element (len(data)).
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
    # period_samples can be float.
    # Window size should be approx 4 periods.
    window_samples = int(period_samples * 4)
    
    if head < window_samples:
        return 0.0
        
    start_idx = head - window_samples
    window_data = data[start_idx : head]
    
    I = 0.0
    Q = 0.0
    
    # Omega must be exact based on the sub-sample period
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
        
    phi = math.atan2(-Q, I)
    
    if phi < 0:
        phi += 2.0 * math.pi
        
    return phi

def run_rotation_task(data, head, state):
    count = head
    if count < CORR_WINDOW * 2:
        return

    # 1. Coarse Search
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

    # 2. Process Slopes
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
        
        # Sub-sample interpolation
        # Using Parabolic Interpolation:
        # y = ax^2 + bx + c
        # We have 3 points: (x-1, y_minus), (x, y_0), (x+1, y_plus)
        # The minimum is at x - (y_plus - y_minus) / (2 * (y_plus - 2*y_0 + y_minus))
        
        delta = 0.0
        if real_final_lag > START_LAG and real_final_lag < END_LAG:
            err_minus = calculate_autocorr_error(data, head, real_final_lag - 1, CORR_WINDOW)
            err_plus = calculate_autocorr_error(data, head, real_final_lag + 1, CORR_WINDOW)
            
            numerator = err_minus - err_plus
            denominator = 2 * (err_minus - 2 * fine_min + err_plus)
            
            if denominator != 0:
                delta = numerator / denominator
                
        state.valid_lock = True
        state.estimated_period_us = (final_lag + delta) * INTERPOLATION_INTERVAL_US
        state.rotation_rate = 1000000.0 / state.estimated_period_us

def run_rotation_task_nufft(raw_ts_window, raw_rssi_window, state):
    # raw_ts_window: list of timestamps in us
    # raw_rssi_window: list of RSSI values
    
    if len(raw_ts_window) < 100:
        return

    # Convert to numpy arrays
    t = np.array(raw_ts_window, dtype=np.float64)
    y = np.array(raw_rssi_window, dtype=np.complex128) # Signal is real, but finufft takes complex
    
    # Normalize time to start at 0 for phase consistency relative to window start
    # t = (t - t[0]) / 1000000.0 # seconds
    # Actually, keep it absolute but scaled to seconds for frequency calculation
    # But usually for Type 3, we want the "target frequencies"
    t_sec = t / 1_000_000.0
    
    # Define candidate frequencies
    # Lags: 200us to 1000us -> Periods: 20ms to 100ms
    # Freqs: 10Hz to 50Hz
    # Step lag 5us is approx fine.
    # We will search the same "lag" space as the time-domain algo for consistency
    
    lags = np.arange(START_LAG, END_LAG + 1, STEP_LAG) # in 100us units (from original algo)
    # Original algo lags are effectively indices into a 100us sampled array.
    # So lag=200 means 200 * 100us = 20000us = 20ms.
    periods_us = lags * INTERPOLATION_INTERVAL_US
    periods_sec = periods_us / 1_000_000.0
    freqs_hz = 1.0 / periods_sec
    omegas = 2 * np.pi * freqs_hz
    
    # NUFFT Type 3: inputs (t, y), request transform at frequencies (omegas)
    # F[k] = sum(c[j] * exp(1i * s[k] * x[j]))
    # Here x[j] is time t, s[k] is omega. 
    # Usually Fourier transform is sum(f(t) * exp(-i * omega * t))
    # finufft sign convention: +i (default) or -i
    # We'll use -1 (isign) for standard forward transform convention if we want physical meaning,
    # but for magnitude peak it doesn't matter much. Let's use -1.
    
    f = finufft.nufft1d3(t_sec, y, omegas, isign=-1, eps=1e-6)
    
    # Find peak magnitude
    mags = np.abs(f)
    peak_idx = np.argmax(mags)
    peak_omega = omegas[peak_idx]
    
    # Refine peak using parabolic interpolation on the magnitude spectrum?
    # Or just take the max for now. Let's start with max.
    
    # Update State
    state.rotation_rate = peak_omega / (2 * np.pi)
    state.estimated_period_us = 1_000_000.0 / state.rotation_rate
    state.valid_lock = True # Assuming we always find "some" peak
    
    # Only trust it if peak is significant? (Skip for now, simplistic)
    if mags[peak_idx] < 100: # Arbitrary threshold
         state.valid_lock = False



def main():
    parser = argparse.ArgumentParser(description='Simulate rotation tracking with interpolation strategies.')
    parser.add_argument('file', help='CSV dump file')
    parser.add_argument('--strategy', choices=['baseline', 'linear', 'smart', 'nufft'], default='baseline', help='Interpolation strategy')
    
    args = parser.parse_args()
    csv_filename = args.file
    strategy = args.strategy
    
    print(f"Loading {csv_filename}...")
    try:
        raw_ts, raw_rssi = load_raw_data(csv_filename)
        if not raw_ts:
            print("No data loaded")
            return
    except FileNotFoundError:
        print(f"Error: {csv_filename} not found.")
        return
        
    print(f"Loaded {len(raw_rssi)} raw samples. Running {strategy} strategy...")
    
    state = RotationState()
    
    # Output buffer (we build this incrementally)
    interp_timestamps = []
    interp_rssi = []
    
    # Analysis outputs
    out_timestamps = []
    out_rate = []
    out_phase = []
    out_lock = []
    
    start_time = raw_ts[0]
    end_time = raw_ts[-1]
    
    current_time = start_time
    raw_idx = 0
    
    # Current simulation buffer (growing)
    # Note: interp_rssi IS the buffer
    
    UPDATE_INTERVAL = 100 # run algo every 100 samples (10ms)
    
    while current_time <= end_time:
        # 1. INTERPOLATION LOGIC
        
        # Advance raw_idx such that we know the surrounding raw points
        # raw_idx points to the last sample <= current_time
        while raw_idx < len(raw_ts) - 1 and raw_ts[raw_idx + 1] <= current_time:
            raw_idx += 1
            
        this_val = raw_rssi[raw_idx]
        
        if strategy == 'baseline':
            # Sample and Hold
            val = this_val
            
        elif strategy == 'linear' or strategy == 'smart':
            # Need next sample
            if raw_idx < len(raw_ts) - 1:
                prev_ts = raw_ts[raw_idx]
                prev_val = raw_rssi[raw_idx]
                next_ts = raw_ts[raw_idx+1]
                next_val = raw_rssi[raw_idx+1]
                
                gap = next_ts - prev_ts
                
                # Check for Smart Healing condition
                is_gap = gap > 300
                healed = False
                
                if strategy == 'smart' and is_gap and state.valid_lock:
                     # Try to heal from history
                     period_samples = int(state.estimated_period_us / INTERPOLATION_INTERVAL_US)
                     # We need history at current_time - period
                     # Since interp_rssi is filled up to current index (which effectively maps to current_time - step)
                     # We are calculating the value FOR current_time.
                     # Index 0 corresponds to start_time. 
                     # Current index is len(interp_rssi).
                     curr_idx = len(interp_rssi)
                     hist_idx = curr_idx - period_samples
                     
                     if hist_idx >= 0 and hist_idx < len(interp_rssi):
                         val = interp_rssi[hist_idx]
                         healed = True
                
                if not healed:
                     # Linear Interpolation
                     if next_ts > prev_ts: # Avoid div/0
                        ratio = (current_time - prev_ts) / (next_ts - prev_ts)
                        val = prev_val + (next_val - prev_val) * ratio
                     else:
                        val = prev_val
            else:
                val = this_val
        
        elif strategy == 'nufft':
            # For NUFFT, we don't strictly need interpolation for the RATE estimation,
            # but we still need to plot "something" or at least advance time.
            # We'll just replicate baseline (sample-and-hold) for the visualization RSSI trace.
            val = this_val
        
        interp_timestamps.append(current_time)
        interp_rssi.append(val)
        
        # 2. ALGO EXECUTION
        curr_idx = len(interp_timestamps) - 1
        
        if curr_idx >= 2000 and curr_idx % UPDATE_INTERVAL == 0:
            if strategy == 'nufft':
                # Pass the raw window relative to current time
                # We need raw samples within [current_time - window, current_time]
                # Window size 2000 * 100us = 200ms? 
                # OR user said "window size of 2000" -- usually means 2000 samples in the interpolated array?
                # The interpolated array has 100us step. So 2000 * 100us = 0.2s = 200ms.
                # Let's find raw samples in the last 200ms.
                
                win_duration_us = 2000 * INTERPOLATION_INTERVAL_US
                win_start_time = current_time - win_duration_us
                
                # Extract raw slice (inefficient linear search, but fine for sim)
                # raw_ts is sorted.
                
                # Find range
                # Bisect would be faster, but let's just create a view
                
                # We need to efficiently grab the slice.
                # Since raw_idx tracks current_time, we can scan backwards from raw_idx
                
                slice_ts = []
                slice_rssi = []
                
                # Scan backwards from raw_idx
                # raw_idx points to sample <= current_time
                k = raw_idx
                while k >= 0:
                    t = raw_ts[k]
                    if t < win_start_time:
                        break
                    slice_ts.append(t)
                    slice_rssi.append(raw_rssi[k])
                    k -= 1
                
                # Re-reverse to be chronological
                slice_ts.reverse()
                slice_rssi.reverse()
                
                run_rotation_task_nufft(slice_ts, slice_rssi, state)
                
            else:
                run_rotation_task(interp_rssi, curr_idx, state)
            
        # 3. PHASE CALCULATION
        phase = 0.0
        if state.valid_lock:
            # Pass float for precision
            period_samples_float = state.estimated_period_us / INTERPOLATION_INTERVAL_US
            if curr_idx >= int(period_samples_float):
                phase = calculate_iq_phase(interp_rssi, curr_idx, period_samples_float)
                
        # Store output
        out_timestamps.append(current_time)
        out_rate.append(state.rotation_rate)
        out_phase.append(phase)
        out_lock.append(1 if state.valid_lock else 0)
        
        current_time += INTERPOLATION_INTERVAL_US

    print("Simulation Complete. Plotting...")
    
    # Plotting
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 1. RSSI
    ax1.plot(interp_timestamps, interp_rssi, 'b-', label='Interpolated RSSI', linewidth=0.5)
    # Overlay raw points 
    ax1.plot(raw_ts, raw_rssi, 'r.', label='Raw Samples', markersize=2, alpha=0.5)
    
    ax1.set_ylabel('RSSI (dBm)')
    ax1.set_title(f'Tracking ({os.path.basename(csv_filename)}) - {strategy.upper()}')
    ax1.legend(loc='upper right')
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
    for k in range(1, len(out_phase)):
        if out_phase[k] < out_phase[k-1] - math.pi:
            ax1.axvline(x=out_timestamps[k], color='orange', alpha=0.3, linestyle='--')
            
    ax3.set_xlabel('Timestamp (us)')
    plt.tight_layout()
    
    base_name = os.path.splitext(os.path.basename(csv_filename))[0]
    out_png = f"simulation_{base_name}_{strategy}.png"
    
    plt.savefig(out_png, dpi=300)
    print(f"Saved {out_png}")

if __name__ == "__main__":
    main()
