import pandas as pd
import numpy as np
import os
from pathlib import Path

# Config
INPUT_FILES = ["csi_log.csv", "csi_log2.csv"]
OUTPUT_DIR = "cleaned_runs"
KEEP_COLS = ["local_timestamp", "rssi", "tmag_x", "tmag_y", "tmag_z"]
GAP_THRESHOLD_US = 500000  # 0.5 seconds in microseconds (timestamps seem to be us based on usage in C code)
# Note: In C code, esp_timer_get_time() returns microseconds.
# Checking log snippet: 4009193441 -> 4009s? Or is it cycles? 
# esp_timer_get_time() returns int64_t time in microseconds since boot.
# 4009193441 us = ~4009 seconds = ~66 minutes. Plausible.

def clean_and_split(file_path):
    print(f"Processing {file_path}...")
    try:
        df = pd.read_csv(file_path, usecols=KEEP_COLS)
    except ValueError as e:
        print(f"Error reading {file_path}: {e}")
        # Try finding actual columns if some are missing or named differently
        preview = pd.read_csv(file_path, nrows=1)
        print(f"Columns found: {preview.columns.tolist()}")
        return

    # Sort just in case, though logs should be append-only
    # df = df.sort_values("local_timestamp").reset_index(drop=True) 
    # Actually, don't sort yet, outlier detection depends on sequence.

    clean_rows = []
    
    # We will iterate and build a list of valid files
    # A "run" is a sequence of timestamps with no large gaps
    
    current_run = []
    
    # Simple Outlier & Split Logic
    # We need to look ahead, so it's easier to just do a loop or Use logic on the diffs
    # Re-implementing specific "Jump up and return" logic requires window of 3.
    
    # Convert to numpy for speed
    timestamps = df["local_timestamp"].values
    rssis = df["rssi"].values
    tmags_x = df["tmag_x"].values
    tmags_y = df["tmag_y"].values
    tmags_z = df["tmag_z"].values
    
    n = len(timestamps)
    
    # Store indices of valid points
    valid_indices = []
    
    # Pass 1: Identify valid points (Filter Outliers)
    # Outlier definition: T[i] >> T[i-1] AND T[i+1] ~ T[i-1]
    # We essentially want to ignore T[i] if it's a spike.
    
    # Actually, simpler approach:
    # Iterate through. Keep track of "current stable timestamp".
    # If new_ts is close to current_stable + expected_dt, calculate dt.
    # If new_ts is WAY larger (huge gap), it might be a split OR an outlier.
    # If outlier: the NEXT point will be back near current_stable.
    # If split: the NEXT point will be near new_ts.
    
    i = 0
    start_new_file = True
    
    run_id = 0
    file_prefix = Path(file_path).stem
    
    # Prepare output dir
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    current_run_data = [] # List of tuples/rows
    
    last_valid_ts = timestamps[0]
    current_run_data.append( (timestamps[0], rssis[0], tmags_x[0], tmags_y[0], tmags_z[0]) )
    
    for i in range(1, n):
        curr_ts = timestamps[i]
        diff = curr_ts - last_valid_ts
        
        # Check for negative diff (restart?)
        if diff < 0:
            # Timestamp went backwards. Definitely a new run or weirdness.
            # Treat as split.
            # But wait, is it just one point back in time (noise) or full reset?
            # e.g. 100, 101, 5, 6, 7... -> Reset
            # e.g. 100, 101, 99, 102... -> Jitter
            
            # Look ahead to decide
            if i + 1 < n:
                next_ts = timestamps[i+1]
                if abs(next_ts - curr_ts) < GAP_THRESHOLD_US:
                    # The flow continues from the lower value -> Reset
                    save_run(current_run_data, file_prefix, run_id)
                    run_id += 1
                    current_run_data = []
                    last_valid_ts = curr_ts
                    current_run_data.append((curr_ts, rssis[i], tmags_x[i], tmags_y[i], tmags_z[i]))
                    continue
                else:
                    # Next one is big again? 
                    # 100, 5, 101 -> 5 is outlier low.
                    pass # Ignore curr_ts
            else:
                # Last point is low? Ignore.
                pass
            continue

        if diff > GAP_THRESHOLD_US:
            # Potential gap or outlier spike
            # Check next point
            if i + 1 < n:
                next_ts = timestamps[i+1]
                diff_next = next_ts - last_valid_ts
                
                # If next point acts as if gap didn't happen (relative to last_valid)
                if abs(diff_next) < GAP_THRESHOLD_US:
                    # It was a spike! Ignore curr_ts.
                    # print(f"Skipping spike at index {i}: {curr_ts} (prev {last_valid_ts}, next {next_ts})")
                    continue 
                else:
                    # The next point also confirms the jump (or is a new jump)
                    # Likely a real gap/split.
                    # Save current run
                    save_run(current_run_data, file_prefix, run_id)
                    run_id += 1
                    current_run_data = []
                    last_valid_ts = curr_ts
                    current_run_data.append((curr_ts, rssis[i], tmags_x[i], tmags_y[i], tmags_z[i]))
            else:
                # End of file with a gap. Just add it? Or discard single point?
                # Discard single point at end to be safe.
                pass
        else:
            # Normal point
            last_valid_ts = curr_ts
            current_run_data.append((curr_ts, rssis[i], tmags_x[i], tmags_y[i], tmags_z[i]))

    # Save final run
    if current_run_data:
        save_run(current_run_data, file_prefix, run_id)

def save_run(data, prefix, run_id):
    if not data:
        return
    if len(data) < 100: # Ignore tiny runs
        return
        
    df_out = pd.DataFrame(data, columns=KEEP_COLS)
    output_path = os.path.join(OUTPUT_DIR, f"{prefix}_part_{run_id}.csv")
    print(f"Saving {output_path} with {len(df_out)} rows.")
    df_out.to_csv(output_path, index=False)

if __name__ == "__main__":
    for f in INPUT_FILES:
        if os.path.exists(f):
            clean_and_split(f)
        else:
            print(f"File {f} not found.")
