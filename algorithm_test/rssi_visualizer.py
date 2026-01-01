import csv
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def visualize_enhanced_csi(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    rssi_values = []
    # csi_timestamps = []
    tmag_x_vals = []
    tmag_y_vals = []
    tmag_z_vals = []
    avg_amplitudes = []

    print(f"Reading {csv_path}...")
    try:
        with open(csv_path, mode='r') as f:
            reader = csv.DictReader(f)
            
            # Identify CSI columns once
            fieldnames = reader.fieldnames
            i_cols = sorted([col for col in fieldnames if col.startswith('csi_') and col.endswith('_i')])
            q_cols = sorted([col for col in fieldnames if col.startswith('csi_') and col.endswith('_q')])
            
            if not i_cols or not q_cols:
                print("Warning: No CSI I/Q columns found in CSV.")
            
            for row in reader:
                try:
                    # Basic metrics
                    rssi = float(row['rssi'])
                    # csi_ts = float(row['csi_timestamp']) # unused
                    
                    tmag_x = float(row['tmag_x'])
                    tmag_y = float(row['tmag_y'])
                    tmag_z = float(row['tmag_z'])
                    
                    # Calculate CSI Amplitude
                    if i_cols:
                        i_vals = np.array([float(row[c]) for c in i_cols])
                        q_vals = np.array([float(row[c]) for c in q_cols])
                        # Amplitude = sqrt(I^2 + Q^2)
                        amp = np.sqrt(i_vals**2 + q_vals**2)
                        avg_amp = np.mean(amp)
                    else:
                        avg_amp = 0
                        
                    rssi_values.append(rssi)
                    # csi_timestamps.append(csi_ts)
                    tmag_x_vals.append(tmag_x)
                    tmag_y_vals.append(tmag_y)
                    tmag_z_vals.append(tmag_z)
                    avg_amplitudes.append(avg_amp)
                    
                except (ValueError, KeyError):
                    continue
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    if not rssi_values:
        print("No valid data found in CSV.")
        return

    # Convert to numpy arrays for easier manipulation
    rssi_values = np.array(rssi_values)
    tmag_x_vals = np.array(tmag_x_vals)
    tmag_y_vals = np.array(tmag_y_vals)
    tmag_z_vals = np.array(tmag_z_vals)
    avg_amplitudes = np.array(avg_amplitudes)
    indices = np.arange(len(rssi_values))

    # Normalize timestamps to start at 0 and convert to seconds if possible
    # (Optional, but usually better for plotting raw vs packet index)
    # The user asked for "csi timestamp", let's show it relative to start
    # norm_csi_ts = (csi_timestamps - csi_timestamps[0]) / 1_000_000.0

    print(f"Plotting {len(rssi_values)} data points...")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    # Plot RSSI
    ax1.plot(indices, rssi_values, color='#007acc', linewidth=0.5, alpha=0.6, label='Raw RSSI')
    if len(rssi_values) > 50:
        window = 50
        rssi_smooth = np.convolve(rssi_values, np.ones(window)/window, mode='valid')
        ax1.plot(indices[window-1:], rssi_smooth, color='#e31a1c', linewidth=1.5, label='50-pt Smooth')
    ax1.set_ylabel('RSSI (dBm)')
    ax1.set_title('RSSI over Packet Index')
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')

    # Plot TMAG Data
    ax2.plot(indices, tmag_x_vals, color='r', linewidth=1, label='TMAG X', alpha=0.7)
    ax2.plot(indices, tmag_y_vals, color='g', linewidth=1, label='TMAG Y', alpha=0.7)
    ax2.plot(indices, tmag_z_vals, color='b', linewidth=1, label='TMAG Z', alpha=0.7)
    ax2.set_ylabel('Field Strength (items)')
    ax2.set_title('TMAG X, Y, Z over Packet Index')
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper left')

    # Plot Average CSI Amplitude
    ax3.plot(indices, avg_amplitudes, color='#ff7f00', linewidth=0.5, alpha=0.6, label='Raw Avg Amplitude')
    if len(avg_amplitudes) > 50:
        window = 50
        amp_smooth = np.convolve(avg_amplitudes, np.ones(window)/window, mode='valid')
        ax3.plot(indices[window-1:], amp_smooth, color='#6a3d9a', linewidth=1.5, label='50-pt Smooth')
    ax3.set_ylabel('Amplitude')
    ax3.set_xlabel('Packet Index')
    ax3.set_title('Average CSI Amplitude over Packet Index')
    ax3.grid(True, linestyle='--', alpha=0.5)
    ax3.legend(loc='upper right')

    # General styling
    for ax in [ax1, ax2, ax3]:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    plt.tight_layout()
    print("Close the plot window to exit.")
    plt.show()

if __name__ == "__main__":
    csv_file = "csi_log.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    visualize_enhanced_csi(csv_file)
