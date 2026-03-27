import struct
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def analyze_pipeline(file_path, output_name="pipeline_deltas"):
    packet_size = 131
    magic_val = 0xDEADBEEF
    
    local_timestamps = []
    sender_timestamps = []
    
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    chunk_size = 65536  # 64KB
    buffer = b""
    with open(file_path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            buffer += chunk
            
            offset = 0
            while offset + packet_size <= len(buffer):
                magic_idx = buffer.find(struct.pack("<I", magic_val), offset)
                if magic_idx == -1:
                    offset = len(buffer) - packet_size
                    break

                if magic_idx + packet_size > len(buffer):
                    offset = magic_idx
                    break

                packet_data = buffer[magic_idx:magic_idx + packet_size]
                magic, local_ts, sender_ts = struct.unpack_from("<Iqq", packet_data)

                local_timestamps.append(local_ts)
                sender_timestamps.append(sender_ts)
                offset = magic_idx + packet_size

            buffer = buffer[offset:]
            
    if not local_timestamps:
        print("No valid packets found.")
        return

    local_ts_array = np.array(local_timestamps)
    sender_ts_array = np.array(sender_timestamps)
    
    local_deltas = np.diff(local_ts_array)
    sender_deltas = np.diff(sender_ts_array)
    
    # Filter for stats
    valid_mask = (local_deltas > 0) & (local_deltas < 1000000)
    f_local = local_deltas[valid_mask]
    
    valid_mask_s = (sender_deltas > 0) & (sender_deltas < 1000000)
    f_sender = sender_deltas[valid_mask_s]
    
    print(f"Analyzed {len(local_timestamps)} packets from {file_path}.")
    
    if len(f_local) > 0:
        print(f"Local TS Deltas (us): Mean={np.mean(f_local):.1f}, Median={np.median(f_local):.1f}, Std={np.std(f_local):.1f}")
    if len(f_sender) > 0:
        print(f"Sender TS Deltas (us): Mean={np.mean(f_sender):.1f}, Median={np.median(f_sender):.1f}, Std={np.std(f_sender):.1f}")

    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    ax1.plot(local_deltas, label="Local Delta (Receiver Time)", alpha=0.7)
    ax1.plot(sender_deltas, label="Sender Delta (Sender Time)", alpha=0.7)
    ax1.set_ylim(0, 5000) # Zoomed in
    ax1.axhline(y=500, color='r', linestyle='--', label="Target 500us")
    ax1.set_title(f"Timestamp Deltas ({file_path})")
    ax1.set_ylabel("Delta (us)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.hist(f_local, bins=100, alpha=0.5, label="Local Delta")
    ax2.hist(f_sender, bins=100, alpha=0.5, label="Sender Delta")
    ax2.axvline(x=500, color='r', linestyle='--')
    ax2.set_title("Delta Distribution (Zoomed < 1s)")
    ax2.set_xlabel("Delta (us)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    artifact_dir = "/home/t/.gemini/antigravity/brain/a1a62c64-d7cb-4e5f-807c-93db7ca2ee36"
    artifact_plot_path = os.path.join(artifact_dir, f"{output_name}.png")
    plt.savefig(artifact_plot_path)
    print(f"Plot saved to {artifact_plot_path}")

if __name__ == "__main__":
    file_to_analyze = sys.argv[1] if len(sys.argv) > 1 else "/home/t/esp/rotation_sensing/receiver/test_pipeline.bin"
    output_base = sys.argv[2] if len(sys.argv) > 2 else "pipeline_deltas"
    analyze_pipeline(file_to_analyze, output_base)
