import serial
import struct
import time
import argparse
import csv
import sys

# Constants from C code
CSI_LOG_MAGIC = 0xDEADBEEF
PACKET_SIZE = 131 
HEADER_SIZE = 37
CSI_DATA_SIZE = 94 # 47 subcarriers * 2 bytes (I, Q)

# Packet structure: <I (magic) q (local_ts) q (sender_ts) h (x) h (y) h (z) b (rssi) H (csi_len) q (csi_ts)
# I: unsigned int (4), q: long long (8), h: short (2), b: signed char (1), H: unsigned short (2)
HEADER_STRUCT = "<IqqhhhbHq"

def main():
    parser = argparse.ArgumentParser(description="ESP32 CSI CSV Logger")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate (ignored for USB-JTAG)")
    parser.add_argument("--output", default="csi_log.csv", help="Output file")
    args = parser.parse_args()

    print(f"Opening port {args.port}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        # Reset sequence
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        ser.dtr = False
        time.sleep(1.0) # Wait for boot
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    print(f"Logging to {args.output}...")
    
    # Prepare CSV header
    # Header fields
    header_fields = ["local_timestamp", "sender_timestamp", "tmag_x", "tmag_y", "tmag_z", "rssi", "csi_len", "csi_timestamp"]
    # CSI fields: csi_0_i, csi_0_q, ... csi_46_i, csi_46_q
    csi_fields = []
    for i in range(47):
        csi_fields.append(f"csi_{i}_i")
        csi_fields.append(f"csi_{i}_q")
    
    csv_headers = header_fields + csi_fields

    count = 0
    start_time = time.time()
    
    with open(args.output, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)
        
        buffer = bytearray()
        try:
            while True:
                try:
                    data = ser.read(4096)
                except serial.SerialException:
                    continue
                if not data:
                    continue
                
                buffer.extend(data)
                
                while len(buffer) >= PACKET_SIZE:
                    # Search for Magic Word
                    magic_idx = buffer.find(struct.pack("<I", CSI_LOG_MAGIC))
                    if magic_idx == -1:
                        # Keep last 3 bytes just in case magic word is split
                        if len(buffer) > 3:
                            buffer = buffer[-3:]
                        else:
                            # If buffer is small and no magic, keep it all
                            pass 
                        break
                    
                    if magic_idx > 0:
                        buffer = buffer[magic_idx:]
                    
                    if len(buffer) < PACKET_SIZE:
                        break
                    
                    packet_data = buffer[:PACKET_SIZE]
                    
                    # Parse Packet
                    header_data = packet_data[:HEADER_SIZE]
                    csi_data_raw = packet_data[HEADER_SIZE:HEADER_SIZE+CSI_DATA_SIZE]

                    try:
                        unpacked = struct.unpack(HEADER_STRUCT, header_data)
                        # unpacked: (magic, local_ts, sender_ts, x, y, z, rssi, csi_len, csi_ts)
                        # We skip magic (index 0)
                        
                        row_data = list(unpacked[1:]) 
                        
                        # Parse CSI Data (signed 8-bit integers)
                        # stored as I, Q interleaved
                        for i in range(0, CSI_DATA_SIZE, 2):
                            i_val = int.from_bytes(csi_data_raw[i:i+1], byteorder='little', signed=True)
                            q_val = int.from_bytes(csi_data_raw[i+1:i+2], byteorder='little', signed=True)
                            row_data.append(i_val)
                            row_data.append(q_val)
                            
                        writer.writerow(row_data)
                        count += 1
                        
                        if count % 100 == 0:
                            elapsed = time.time() - start_time
                            pps = count / elapsed if elapsed > 0 else 0
                            
                            sender_ts = unpacked[2]
                            tmag_x = unpacked[3]
                            tmag_y = unpacked[4]
                            tmag_z = unpacked[5]
                            rssi = unpacked[6]
                            
                            print(f"\rCaptured {count} pkts ({pps:.1f} p/s) | TS: {sender_ts} | RSSI: {rssi} | TMAG: {tmag_x}, {tmag_y}, {tmag_z}   ", end="", flush=True)
                            # f.flush() # Optional: flush periodically if needed immediately
                            
                    except struct.error as e:
                        print(f"\nError unpacking packet: {e}")
                    
                    # Remove processed packet
                    buffer = buffer[PACKET_SIZE:]

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
