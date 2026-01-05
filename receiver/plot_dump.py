#!/usr/bin/env python3
import csv
import argparse
import matplotlib.pyplot as plt
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Plot RSSI from dump CSV")
    parser.add_argument("file", help="Path to CSV file (e.g., dump_slot04.csv)")
    parser.add_argument("--sort", action="store_true", help="Sort by timestamp (fixes wrap-around visual artifacts)")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: File {args.file} not found.")
        sys.exit(1)

    timestamps = []
    rssi_values = []

    print(f"Reading {args.file}...")
    try:
        with open(args.file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ts = float(row["Timestamp_US"])
                    rssi = float(row["RSSI"])
                    timestamps.append(ts)
                    rssi_values.append(rssi)
                except ValueError:
                    continue
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    if not timestamps:
        print("No valid data found.")
        return

    # Convert to seconds relative to start
    # If sorting is requested
    if args.sort:
        combined = sorted(zip(timestamps, rssi_values))
        timestamps, rssi_values = zip(*combined)

    start_time = timestamps[0]
    time_sec = [(t - start_time) / 1e6 for t in timestamps]

    # Calculate Deltas
    deltas = []
    if len(timestamps) > 1:
        for i in range(1, len(timestamps)):
            d = timestamps[i] - timestamps[i-1]
            # Handle wrap-around or huge gaps if not checking for them, 
            # but usually for a dump we expect continuous-ish data.
            # If sorted, d should be positive.
            deltas.append(d)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=False)

    # Plot 1: RSSI vs Time
    ax1.plot(time_sec, rssi_values, label="RSSI", linewidth=1, marker='.', markersize=2)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("RSSI (dBm)")
    ax1.set_title(f"RSSI over Time: {os.path.basename(args.file)}")
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()

    # Plot 2: Delta vs Index
    # We have N timestamps, N-1 deltas.
    # Indices 1 to N-1
    indices = range(1, len(timestamps))
    ax2.plot(indices, deltas, label="Delta (us)", color='orange', linewidth=1, marker='.', markersize=2)
    ax2.set_xlabel("Sample Index")
    ax2.set_ylabel("Delta (us)")
    ax2.set_title("Timestamp Delta vs Index")
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()
    
    plt.tight_layout()
    
    # Save plot
    output_png = args.file.replace(".csv", ".png")
    plt.savefig(output_png)
    print(f"Plot saved to {output_png}")
    
    # Show plot (optional, might not work in headless env, but good to have)
    # plt.show()

if __name__ == "__main__":
    main()
