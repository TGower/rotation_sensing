import pandas as pd
import numpy as np
import glob
import os
import matplotlib.pyplot as plt
import scipy.signal

# Configuration matches C code
INTERPOLATION_INTERVAL_US = 100  # 10kHz
SMOOTHING_WINDOW = 1             # Default in C code was 1 unless config changed. The user prompt mentioned smoothing_window in context of previous convos, but in the C file I read earlier: `.smoothing_window = 1`.
CORRELATION_WINDOW = 1000        # samples (100ms at 10kHz?) No, C code says: `if (w_len < 1) w_len = 1;` 
# Wait, "INTERPOLATION_INTERVAL_US 100" means 100us per sample.
# 1000 samples * 100us = 100,000us = 100ms.
# C code rotation_task: `start_lag = 200` (20ms), `end_lag = 3000` (300ms).
# `step_lag = 5`, `step_window = 5`.

STEP_LAG = 5
STEP_WINDOW = 5
START_LAG = 200
END_LAG = 3000

INPUT_DIR = "cleaned_runs"

def interpolate_signal(timestamps, rssi_values, smoothing_window=1):
    """
    Simulates the C code's interpolate_rssi function.
    Resamples non-uniform timestamps to fixed INTERPOLATION_INTERVAL_US grid.
    Applies sliding window smoothing.
    """
    if len(timestamps) < 2:
        return np.array([]), np.array([])

    t_start = timestamps[0]
    t_end = timestamps[-1]
    
    # Generate target timestamps
    target_ts = np.arange(t_start, t_end, INTERPOLATION_INTERVAL_US)
    
    output_rssi = []
    
    # We can use numpy/pandas for faster processing than simulating the loop exactly step-by-step
    # but to be essentially identical to C, we should be careful.
    # The C code uses "Nearest Neighbor" for the raw value: `int8_t val = rssi;` (from the latched callback value)
    # Actually, in C: `while (target_ts <= timestamp) { int8_t val = rssi; ... }`
    # It takes the *current* received packet's RSSI for all interpolation points until that packet's timestamp is passed.
    # This is effectively "Previous Value" or "Current Value" hold?
    # Logic:
    # 1. Packet arrives at T_now with RSSI_new.
    # 2. Last buffer time was T_last.
    # 3. While T_last + 100us <= T_now: add RSSI_new to buffer.
    # So it behaves like a "Next Value" hold/fill. It backfills the gap with the NEW value.
    
    current_idx = 0
    
    interpolated_rssi = []
    interpolated_ts = []
    
    # Sliding window state
    window = []
    window_sum = 0
    
    # We iterate through the generated target_ts
    # For each T_target, we need the RSSI of the packet that *covers* it.
    # In C: `while (target_ts <= timestamp)` inside the receive callback.
    # So for a packet at T_i, it fills all T_target where T_target <= T_i (and T_target > T_{i-1}).
    
    # Let's do this efficiently.
    # Create an array of sample values corresponding to the nearest *future* packet (or current).
    # Since we have the full trace, we can use searchsorted.
    
    indices = np.searchsorted(timestamps, target_ts, side='left')
    # searchsorted 'left': indices[i] is such that timestamps[indices[i]-1] < target_ts[i] <= timestamps[indices[i]]
    # This means timestamps[indices[i]] is the first timestamp >= target_ts[i].
    # This matches `target_ts <= timestamp` filling loop.
    indices = np.clip(indices, 0, len(timestamps) - 1)
    
    raw_resampled_rssi = rssi_values[indices]
    
    # Now apply smoothing window
    smooth_rssi = []
    
    # Python loop for smoothing is slow, use convolution or pandas rolling
    if smoothing_window <= 1:
        return target_ts, raw_resampled_rssi
        
    s = pd.Series(raw_resampled_rssi)
    # The C code implementation of sliding window:
    # running_sum += val
    # if valid_history > w_len: running_sum -= oldest
    # smoothed = running_sum / valid_history
    # This is a simple moving average.
    
    # Pandas rolling(min_periods=1) matches this growing window behavior at start.
    smooth_rssi = s.rolling(window=smoothing_window, min_periods=1).mean()
    
    return target_ts, smooth_rssi.values

def autocorrelation_estimator(signal, step_lag=STEP_LAG, step_win=STEP_WINDOW):
    """
    Runs the autocorrelation logic: sum(|diff|)
    Returns estimated period in samples (lags).
    """
    # Signal should be long enough (CORRELATION_WINDOW samples)
    # But here we want to run it over the whole signal? 
    # Or mimic the task that runs continuously?
    # Let's run it in a sliding window over the signal every N samples.
    
    # For debugging, let's just evaluate it every 1000 samples (100ms) over the file.
    
    results = []
    
    times = []
    periods = []
    rates = []
    
    n_samples = len(signal)
    
    # Jump by 100 samples (10ms) like the task (approx)
    for i in range(CORRELATION_WINDOW * 2, n_samples, 100):
        # Window end is i
        # We need data from (i - CORRELATION_WINDOW) backwards for correlation?
        # C code: 
        # idx1 = (head - 1 - i ...); 
        # idx2 = (head - 1 - i - lag ...);
        # It uses a window of CORRELATION_WINDOW samples.
        # It compares signal[now - k] with signal[now - k - lag].
        
        # Segment extraction
        # We need indices [i - CORRELATION_WINDOW, i] roughly
        # And we perform checks with lags up to END_LAG (3000).
        # So we need data back to i - CORRELATION_WINDOW - END_LAG.
        
        if i - CORRELATION_WINDOW - END_LAG < 0:
            continue
            
        # Extract relevant chunk to avoid large array indexing in loop
        # We need signal[i - CORRELATION_WINDOW - END_LAG : i]
        # Let's call this buffer
        # In this buffer, "head" is at the end.
        
        min_diff = float('inf')
        best_lag = 0
        
        # Coarse Search
        # optimize: vectorize this
        # range: start_lag to end_lag, step step_lag
        
        # Vectorized Diff Function (MAD - Mean Absolute Difference)
        # We want sum(|x[t] - x[t-lag]|) for t in window
        
        # Slice for the "base" window (idx1 in C)
        # C loop: for i=0; i<corr_window; i+=step_win
        # idx1 = head - 1 - i
        # So it takes points: head-1, head-1-step, head-1-2*step...
        
        # Let's create the base_indices relative to current 'i'
        # base_indices = [i-1, i-1-step, i-1-2*step ...] length is CORR_WINDOW/step_win
        
        base_indices = np.arange(1, CORRELATION_WINDOW + 1, step_win)
        # relative to current head 'i', indices are i - base_indices
        base_vals = signal[i - base_indices]
        
        # Lags to check
        lags = np.arange(START_LAG, END_LAG + 1, step_lag)
        
        # We want to find lag that minimizes sum(|base_vals - delayed_vals|)
        # delayed_vals depends on lag.
        # delayed_indices = i - base_indices - lag
        
        # This double loop is heavy in pure python.
        # With numpy broadcasting:
        # We need a matrix of shifted signals.
        
        # Construct matrix of delayed values
        # Shape: (num_lags, num_window_points)
        # We can construct indices matrix
        
        # indices_matrix = (i - base_indices) - lags[:, None]
        # delayed_vals = signal[indices_matrix]
        # diffs = np.abs(base_vals - delayed_vals)
        # sums = np.sum(diffs, axis=1)
        # best_lag_idx = np.argmin(sums)
        # best_lag = lags[best_lag_idx]
        
        # Implementation
        idx_matrix = (i - base_indices) - lags[:, np.newaxis]
        vals_matrix = signal[idx_matrix] # This might be memory intensive if big, but windows are small
        diffs = np.abs(vals_matrix - base_vals) # broadcast subtraction
        sums = np.sum(diffs, axis=1)
        
        best_idx = np.argmin(sums)
        min_diff = sums[best_idx]
        best_lag = lags[best_idx]
        
        # --- Robustness: Sub-harmonic/Multiple check ---
        # The user observed that we might be picking a multiple (e.g. 3x) of the true period
        # because the difference function is also low at 2T, 3T, etc.
        # We should check L/2, L/3, L/4. If we find a local minimum there that is "close enough" 
        # to the global min, we prefer the shorter period (higher frequency).
        
        TOLERANCE_FACTOR = 1.30 # Allow sub-harmonic to be 30% worse and still be picked
        
        for div in [4, 3, 2]: # Check 4x, 3x, 2x multiples (check smallest period first? No, check divisors)
            target_lag = best_lag / div
            
            # Must be within valid range
            if target_lag < START_LAG:
                continue
                
            # Find closest index in 'lags' array
            # indices in 'lags' are (val - START) / STEP
            # But let's just search the 'lags' array
            target_idx = np.searchsorted(lags, target_lag)
            
            # check neighborhood ( +/- some range) for a local min
            # Look at +/- 5 indices
            lo = max(0, target_idx - 5)
            hi = min(len(diffs), target_idx + 6)
            
            if hi > lo:
                sub_region = sums[lo:hi] # Use 'sums' (1D array), not 'diffs' (2D matrix)
                if len(sub_region) > 0:
                    sub_min_idx = np.argmin(sub_region) # Index within sub_region (0 to len-1)
                    sub_min_diff = sub_region[sub_min_idx]
                    sub_lag = lags[lo + sub_min_idx] # Corresponding lag in original array
                
                # Check 1: Is it a local minimum? (Roughly, yes, argmin in region)
                # Check 2: Is it stronger (lower) or comparable to global min?
                if sub_min_diff < min_diff * TOLERANCE_FACTOR:
                    best_lag = sub_lag
                    min_diff = sub_min_diff
        
        # Re-refine fine search around the (possibly new) best_lag
        final_lag = best_lag # Rough step lag (Wait, we need to fine search around this new lag)
        fine_lags = np.arange(max(START_LAG, best_lag - step_lag), min(END_LAG, best_lag + step_lag) + 1, 1)

        
        idx_matrix_fine = (i - base_indices) - fine_lags[:, np.newaxis]
        vals_matrix_fine = signal[idx_matrix_fine]
        diffs_fine = np.abs(vals_matrix_fine - base_vals)
        sums_fine = np.sum(diffs_fine, axis=1)
        
        final_best_idx = np.argmin(sums_fine)
        final_lag = fine_lags[final_best_idx]
        
        period_us = final_lag * INTERPOLATION_INTERVAL_US
        rate_hz = 1000000.0 / period_us if period_us > 0 else 0
        
        times.append(i * INTERPOLATION_INTERVAL_US) # Time in us
        rates.append(rate_hz)
        
    return times, rates

def get_autocorr_curve(signal, head_idx, correlation_window=CORRELATION_WINDOW):
    """
    Helper to calculate the difference function snapshot for a given signal at a specific head index.
    """
    if head_idx <= correlation_window + END_LAG:
        return None, None
        
    step_lag = STEP_LAG
    step_win = STEP_WINDOW
    
    base_indices = np.arange(1, correlation_window + 1, step_win)
    base_vals = signal[head_idx - base_indices]
    lags = np.arange(START_LAG, END_LAG + 1, step_lag)
    
    idx_matrix = (head_idx - base_indices) - lags[:, np.newaxis]
    vals_matrix = signal[idx_matrix]
    diffs = np.abs(vals_matrix - base_vals)
    sums = np.sum(diffs, axis=1)
    
    return lags, sums

def get_yin_curve(signal, head_idx, correlation_window=CORRELATION_WINDOW):
    """
    Calculates the YIN Cumulative Mean Normalized Difference Function (CMNDF).
    """
    if head_idx <= correlation_window + END_LAG:
        return None, None
        
    step_win = STEP_WINDOW
    # YIN usually needs lags starting from 1 to normalize correctly
    # detailed_lags = np.arange(1, END_LAG + 1, 1) # step 1 is best for YIN
    # But for speed let's use STEP_LAG? No, YIN cumulative sum needs all previous lags? 
    # If we skip lags, the cumulative sum is an approximation.
    # Let's try to use STEP_LAG and approximate (Avg of evaluated lags).
    
    step_lag = STEP_LAG
    lags = np.arange(step_lag, END_LAG + 1, step_lag)
    
    base_indices = np.arange(1, correlation_window + 1, step_win)
    base_vals = signal[head_idx - base_indices]
    
    # Calculate Squared Difference
    idx_matrix = (head_idx - base_indices) - lags[:, np.newaxis]
    vals_matrix = signal[idx_matrix]
    diffs = (vals_matrix - base_vals)**2 # Squared Difference
    sq_sums = np.sum(diffs, axis=1) # Shape: (len(lags),)
    
    # CMNDF Normalization
    # d'(t) = d(t) / [(1/t) * Sum(d(j) for j=1..t)]
    # We have computed d(t) for t in lags.
    # We construct cumulative sum array
    cumulative_sums = np.cumsum(sq_sums)
    # The divisor is average of diffs up to that point.
    # array index i corresponds to lag = lags[i]
    # number of points summed is i+1
    
    avg_sums = cumulative_sums / (np.arange(len(sq_sums)) + 1)
    
    # Handle potentially small divisor (machine epsilon)
    avg_sums[avg_sums == 0] = 1e-10
    
    cmndf = sq_sums / avg_sums
    
    # CMNDF(0) is usually 1, but we start at lag 1 (or 5).
    # If sq_sums[0] is d(min_lag), avg_sums[0] is d(min_lag). Ratio is 1.
    # So the curve typically starts at 1 and drops.
    
    return lags, cmndf

def process_file(filepath):

    print(f"Analyzing {filepath}...")
    try:
        df = pd.read_csv(filepath)
        if len(df) < 50:
            print("Skipping short file.")
            return
            
        timestamps = df["local_timestamp"].values
        rssis = df["rssi"].values
        tmag_x = df["tmag_x"].values
        tmag_y = df["tmag_y"].values
        tmag_z = df["tmag_z"].values
        
        # Interpolate
        # Note: timestamps might wrap or define "0" arbitrarily. 
        # C code `esp_timer_get_time` is strictly increasing since boot.
        
        t_grid, rssi_grid = interpolate_signal(timestamps, rssis, SMOOTHING_WINDOW)
        
        if len(rssi_grid) < CORRELATION_WINDOW + END_LAG + 100:
            print("Not enough interpolated data.")
            return
            
        times, rates = autocorrelation_estimator(rssi_grid)
        
        # Basic stats
        if len(rates) > 0:
            mean_rate = np.mean(rates)
            std_rate = np.std(rates)
            print(f"  Mean Rotation: {mean_rate:.2f} Hz +/- {std_rate:.2f}")
            
            # Debug: Visualization
            
            # Normalize Time to seconds for plotting
            t0 = t_grid[0]
            t_plot = (t_grid - t0) / 1e6
            raw_t_plot = (timestamps - t0) / 1e6
            
            # Calculate Period and Phase
            if mean_rate > 0:
                period_s = 1.0 / mean_rate
            else:
                period_s = 0
            
            # Find a peak in the first few cycles for phase alignment
            # Simple max in first 2 periods
            search_len = int(2 * period_s * 1e6 / INTERPOLATION_INTERVAL_US) if period_s > 0 else 2000
            search_len = min(len(rssi_grid), search_len)
            if search_len > 0:
                peak_idx = np.argmax(rssi_grid[:search_len])
                peak_time_s = t_plot[peak_idx]
            else:
                peak_time_s = 0

            # Calculate smoothed signals needed for plotting and analysis
            windows = [10, 25, 50]
            colors = ['orange', 'green', 'blue']
            smoothed_signals = {}
            for w in windows:
                # Calculate smoothed signal
                smoothed_signals[w] = pd.Series(rssi_grid).rolling(window=w, min_periods=1).mean().values

            # 4. FFT Spectrum
            # Use RAW signal as requested
            fft_sig = rssi_grid 
            # Remove DC
            fft_sig = fft_sig - np.mean(fft_sig)
            # Apply Window function
            window_func = np.hanning(len(fft_sig))
            fft_sig_w = fft_sig * window_func
            
            n_fft = len(fft_sig) * 4 # Zero padding for resolution
            yf = np.fft.rfft(fft_sig_w, n=n_fft)
            xf = np.fft.rfftfreq(n_fft, d=INTERPOLATION_INTERVAL_US/1e6)
            
            mag = np.abs(yf)
            
            # --- Solution A: Harmonic Product Spectrum (HPS) ---
            # Downsample by 2, 3, 4
            hps_spec = np.copy(mag)
            for d in [2, 3, 4]:
                decimated = mag[::d]
                # Pad to match length
                hps_spec[:len(decimated)] *= decimated
                hps_spec[len(decimated):] = 0 # Zero out rest
            
            # --- Solution C: Cepstrum Analysis ---
            # Real Cepstrum = IFFT(log(|FFT|))
            # Use log magnitude
            log_mag = np.log(mag + 1e-10)
            cepstrum = np.abs(np.fft.irfft(log_mag))
            # Quefrency axis corresponds to time lags
            # Do we use rfft or fft? irfft expects rfft output (Hermitian).
            # If we took log of magnitude, we have a real symmetric spectrum (log(|X|)).
            # The IFFT of a real symmetric sequence is real.
            # Using irfft on the positive half (log_mag) is correct if log_mag is treated as the positive half.
            
            n_ceps = len(cepstrum)
            quefrency = np.arange(n_ceps) * (INTERPOLATION_INTERVAL_US/1e6) # Time axis
            
            
            # --- PLOTTING EXPANSION ---
            plt.figure(figsize=(16, 20))
            
            # 1. RSSI (Top Left)
            ax1 = plt.subplot(4, 2, 1)
            ax1.plot(t_plot[:5000], rssi_grid[:5000], label='Original', color='gray', alpha=0.3)
            ax1.plot(t_plot[:5000], smoothed_signals[10][:5000], label='Smoothed (W=10)', color='orange')
            ax1.set_title("1. RSSI Signal")
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 2. TMAG (Top Right)
            ax2 = plt.subplot(4, 2, 2)
            raw_mask = raw_t_plot < t_plot[min(len(t_plot)-1, 5000)]
            ax2.plot(raw_t_plot[raw_mask], tmag_x[raw_mask], label='X', alpha=0.7)
            ax2.plot(raw_t_plot[raw_mask], tmag_y[raw_mask], label='Y', alpha=0.7)
            ax2.plot(raw_t_plot[raw_mask], tmag_z[raw_mask], label='Z', alpha=0.7)
            ax2.set_title("2. Magnetometer")
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # Select a valid index for snapshot
            min_valid_idx = CORRELATION_WINDOW + END_LAG + 1
            if len(rssi_grid) > min_valid_idx:
                plot_idx = max(len(rssi_grid) // 2, min_valid_idx)
                # Ensure it's not out of bounds
                plot_idx = min(plot_idx, len(rssi_grid) - 1)
            else:
                plot_idx = 0 
                
            # 3. Autocorrelation (Middle Left) -> Solution B: First Major Peak
            ax3 = plt.subplot(4, 2, 3)
            # Use Raw diffs
            lags_raw, sums_raw = get_autocorr_curve(rssi_grid, plot_idx)
            if lags_raw is not None:
                t_lags = lags_raw * INTERPOLATION_INTERVAL_US / 1000.0
                ax3.plot(t_lags, sums_raw, color='orange', label='Autocorr Diff (Raw)')
                
                # --- Solution B: First Major Peak Logic ---
                # Find valleys (peaks in inverted signal)
                inv_sums = -sums_raw
                peaks, _ = scipy.signal.find_peaks(inv_sums) # Simple peak finding
                
                # Filter peaks
                # Global Min
                global_min_idx = np.argmin(sums_raw)
                global_min_val = sums_raw[global_min_idx]
                
                # Threshold: anything within 30% of global min is a candidate
                # We sort candidates by time (lag). The first one is the "First Major Peak".
                
                candidates = []
                for p in peaks:
                    if sums_raw[p] < global_min_val * 1.30:
                        candidates.append(p)
                
                if candidates:
                    first_peak_idx = candidates[0]
                    first_peak_ms = t_lags[first_peak_idx]
                    ax3.plot(first_peak_ms, sums_raw[first_peak_idx], 'ro', markersize=8, label=f'First Major: {first_peak_ms:.1f}ms')
                
                # Mark Global Min too
                global_min_ms = t_lags[global_min_idx]
                ax3.plot(global_min_ms, global_min_val, 'bx', markersize=8, label=f'Global Min: {global_min_ms:.1f}ms')

            ax3.set_title("3. Autocorrelation (First Major Peak)")
            ax3.set_xlabel("Lag (ms)")
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            # 4. FFT Spectrum (Middle Right)
            ax4 = plt.subplot(4, 2, 4)
            ax4.plot(xf, mag, color='purple')
            fft_peak_idx = np.argmax(mag)
            fft_peak_freq = xf[fft_peak_idx]
            ax4.set_title(f"4. FFT Spectrum (Raw) (Peak: {fft_peak_freq:.1f} Hz)")
            ax4.set_xlim(0, 100)
            ax4.grid(True, alpha=0.3)
            
            # 5. HPS (Bottom Left) -> Solution A
            ax5 = plt.subplot(4, 2, 5)
            # Normalize HPS
            hps_spec = hps_spec / np.max(hps_spec)
            ax5.plot(xf, hps_spec, color='green')
            hps_peak_idx = np.argmax(hps_spec)
            hps_peak_freq = xf[hps_peak_idx]
            ax5.axvline(x=hps_peak_freq, color='r', linestyle='--', label=f'HPS Peak: {hps_peak_freq:.1f} Hz')
            ax5.set_title("5. Harmonic Product Spectrum (HPS)")
            ax5.set_xlabel("Frequency (Hz)")
            ax5.set_xlim(0, 100)
            ax5.legend()
            ax5.grid(True, alpha=0.3)
            
            # 6. Cepstrum (Bottom Right) -> Solution C
            ax6 = plt.subplot(4, 2, 6)
            # Plot Quefrency in ms (Seconds * 1000)
            quef_ms = quefrency * 1000.0
            # Ignore DC / very low quefrency (e.g. < 10ms)
            mask = (quef_ms > 10) & (quef_ms < 300)
            ax6.plot(quef_ms[mask], cepstrum[mask], color='brown')
            
            if np.any(mask):
                cep_peak_idx_local = np.argmax(cepstrum[mask])
                # map back to full array
                valid_indices = np.where(mask)[0]
                cep_peak_idx = valid_indices[cep_peak_idx_local]
                cep_peak_ms = quef_ms[cep_peak_idx]
                ax6.axvline(x=cep_peak_ms, color='r', linestyle='--', label=f'Cepstrum Peak: {cep_peak_ms:.1f} ms')
                
            ax6.set_title("6. Cepstrum Analysis")
            ax6.set_xlabel("Quefrency (ms)")
            ax6.legend()
            ax6.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            out_img = f"{filepath}_advanced_debug.png"
            # --- Solution D: YIN Algorithm (Bottom Left - Spot 7) ---

            ax7 = plt.subplot(4, 2, 7)
            # Use RAW signal
            yin_lags, yin_cmndf = get_yin_curve(rssi_grid, plot_idx)
            
            if yin_lags is not None:
                yin_t_lags = yin_lags * INTERPOLATION_INTERVAL_US / 1000.0
                ax7.plot(yin_t_lags, yin_cmndf, color='magenta', label='YIN CMNDF')
                ax7.set_title("7. YIN Algorithm (CMNDF)")
                ax7.set_xlabel("Lag (ms)")
                ax7.set_ylabel("Normalized Diff")
                ax7.set_ylim(0, 2.0) # CMNDF usually dips below 1
                ax7.axhline(y=0.1, color='k', linestyle=':', label='Threshold 0.1')
                
                # Find first dip below threshold
                # Or just global min
                min_yin_idx = np.argmin(yin_cmndf)
                min_yin_lag = yin_t_lags[min_yin_idx]
                ax7.plot(min_yin_lag, yin_cmndf[min_yin_idx], 'rx', label=f'Min: {min_yin_lag:.1f} ms')
                
                # First valid dip
                drops = np.where(yin_cmndf < 0.1)[0]
                if len(drops) > 0:
                    first_drop_idx = drops[0]
                    # Find local min in that region?
                    # For simplicity, just mark the first threshold crossing or the min of that lobe
                    first_drop_ms = yin_t_lags[first_drop_idx]
                    ax7.plot(first_drop_ms, yin_cmndf[first_drop_idx], 'go', label=f'First < 0.1: {first_drop_ms:.1f} ms')


            ax7.legend(loc='upper right')
            ax7.grid(True, alpha=0.3)
            
            plt.tight_layout()
            out_img = f"{filepath}_advanced_debug.png"
            plt.savefig(out_img)
            print(f"  Saved advanced plot to {out_img}")
            plt.close()


    except Exception as e:
        print(f"  Error processing: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(INPUT_DIR, "*part_*.csv")))
    # Focus on the larger log2 files first as they likely have the real high speed data
    files = [f for f in files if "csi_log2" in f]
    
    # Take a few interesting ones (larger ones)
    files.sort(key=os.path.getsize, reverse=True)
    
    count = 0
    for f in files:
        if count >= 3: break # Limit to top 3 largest for quick verify
        process_file(f)
        count += 1
