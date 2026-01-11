#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Plot dump CSV with RSSI and Control data")
    parser.add_argument("file", help="Path to CSV file", nargs='?', default="dump_slot01.csv")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: File {args.file} not found.")
        sys.exit(1)

    print(f"Reading {args.file}...")
    try:
        df = pd.read_csv(args.file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        # Fallback hint
        print("Ensure pandas is installed (e.g. source .venv/bin/activate)")
        sys.exit(1)

    # Clean whitespace from columns
    df.columns = df.columns.str.strip()
    
    required_cols = ["Timestamp_US", "RSSI"]
    for col in required_cols:
        if col not in df.columns:
            print(f"Error: Column {col} missing.")
            print(f"Available columns: {df.columns.tolist()}")
            sys.exit(1)

    # Normalize time to seconds starting at 0
    start_ts = df["Timestamp_US"].iloc[0]
    df["Time_Sec"] = (df["Timestamp_US"] - start_ts) / 1e6

    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    
    # Plot 1: RSSI
    ax1 = axes[0]
    ax1.plot(df["Time_Sec"], df["RSSI"], label="RSSI", color='blue', linewidth=1)
    ax1.set_ylabel("RSSI (dBm)")
    ax1.set_title(f"Data dump: {os.path.basename(args.file)}")
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend(loc='upper right')

    # Plot 2: Throttle
    ax2 = axes[1]
    if "Throttle" in df.columns:
        ax2.plot(df["Time_Sec"], df["Throttle"], label="Throttle", color='green', linewidth=1)
        ax2.set_ylabel("Throttle")
        ax2.grid(True, linestyle='--', alpha=0.7)
        ax2.legend(loc='upper right')
    else:
        ax2.text(0.5, 0.5, "No Throttle Data", ha='center', va='center')
        ax2.set_axis_off()

    # Plot 3: Vectors
    ax3 = axes[2]
    has_vector = False
    if "VectorX" in df.columns:
        ax3.plot(df["Time_Sec"], df["VectorX"], label="VectorX", color='red', linewidth=1)
        has_vector = True
    if "VectorY" in df.columns:
        ax3.plot(df["Time_Sec"], df["VectorY"], label="VectorY", color='orange', linewidth=1)
        has_vector = True
    
    if has_vector:
        ax3.set_ylabel("Vector Value")
        ax3.grid(True, linestyle='--', alpha=0.7)
        ax3.legend(loc='upper right')
    else:
        ax3.text(0.5, 0.5, "No Vector Data", ha='center', va='center')
        ax3.set_axis_off()

    ax3.set_xlabel("Time (s)")
    
    # Adjust layout
    plt.tight_layout()
    
    output_png = args.file
    if output_png.lower().endswith('.csv'):
        output_png = output_png[:-4] + '.png'
    else:
        output_png += ".png"
        
    plt.savefig(output_png)
    print(f"Plot saved to {output_png}")

if __name__ == "__main__":
    main()
