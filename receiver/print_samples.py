import struct
import os

def print_samples(file_path, count=3000):
    packet_size = 131
    magic_val = 0xDEADBEEF
    
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    # Header format: magic(I), local_ts(q), sender_ts(q), x(h), y(h), z(h), rssi(b), csi_len(H), csi_ts(q)
    # Total 37 bytes
    header_fmt = "<Iqqhhhbhq"
    
    print(f"{'Index':<6} | {'Local TS':<12} | {'Sender TS':<12} | {'X':<6} | {'Y':<6} | {'Z':<6} | {'RSSI':<5}")
    print("-" * 75)

    samples_printed = 0
    with open(file_path, "rb") as f:
        while samples_printed < count:
            chunk = f.read(packet_size)
            if len(chunk) < packet_size:
                break
            
            magic, local_ts, sender_ts, x, y, z, rssi, csi_len, csi_ts = struct.unpack_from(header_fmt, chunk)
            
            if magic != magic_val:
                # Basic search for next magic if misaligned
                idx = chunk.find(struct.pack("<I", magic_val))
                if idx != -1:
                    f.seek(f.tell() - (packet_size - idx))
                continue
                
            print(f"{samples_printed:<6} | {local_ts:<12} | {sender_ts:<12} | {x:<6} | {y:<6} | {z:<6} | {rssi:<5}")
            samples_printed += 1

if __name__ == "__main__":
    print_samples("/home/t/esp/rotation_sensing/receiver/burst_test.bin")
