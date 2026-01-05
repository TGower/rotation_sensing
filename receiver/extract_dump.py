#!/usr/bin/env python3
import sys
import struct
import csv
import argparse
import os
import subprocess
from datetime import datetime

# Configuration matching Receiver Firmware
RSSI_BUF_SIZE = 6000
SLOT_SIZE = 0x20000     # 128KB
PARTITION_SIZE = 0x200000 # 2MB
PARTITION_OFFSET = 0x187000

# C Struct Layout
# int8_t rssi[6000]
# int64_t timestamp[6000]
# int32_t head
# int32_t tail
# int64_t last_timestamp
STRUCT_FMT = f"<{RSSI_BUF_SIZE}b{RSSI_BUF_SIZE}qiiq"
EXPECTED_SIZE = struct.calcsize(STRUCT_FMT)

def run_esptool_read(port, baud, output_file):
    print(f"Reading full flash partition ({PARTITION_SIZE} bytes) from offset 0x{PARTITION_OFFSET:X}...")
    cmd = [
        "esptool.py",
        "-p", port,
        "-b", str(baud),
        "read_flash",
        str(PARTITION_OFFSET),
        str(PARTITION_SIZE),
        output_file
    ]
    try:
        subprocess.check_call(cmd)
    except FileNotFoundError:
        print("Error: 'esptool.py' not found in PATH.")
        print("Please run: source ~/esp/esp-idf/export.sh")
        print("Or ensure esptool.py is installed and available.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error extracting dump: {e}")
        sys.exit(1)

def parse_slot(slot_index, data):
    # Sanity check: Empty slot (erased flash is 0xFF)
    if data[:4] == b'\xff\xff\xff\xff':
        return None

    if len(data) < EXPECTED_SIZE:
        return None

    try:
        unpacked = struct.unpack(STRUCT_FMT, data[:EXPECTED_SIZE])
        
        rssi_arr = unpacked[0 : RSSI_BUF_SIZE]
        ts_arr = unpacked[RSSI_BUF_SIZE : 2 * RSSI_BUF_SIZE]
        head = unpacked[2 * RSSI_BUF_SIZE]
        tail = unpacked[2 * RSSI_BUF_SIZE + 1]
        last_ts = unpacked[2 * RSSI_BUF_SIZE + 2]

        # Basic validation
        if head < -1 or head >= RSSI_BUF_SIZE:
            return None
        if tail < -1 or tail >= RSSI_BUF_SIZE:
            return None
        
        # If head == -1, buffer is logically empty/uninitialized for our purposes
        if head == -1:
            return None

        csv_filename = f"dump_slot{slot_index:02d}.csv"
        print(f"  Slot {slot_index}: Found valid dump (Head {head}, Tail {tail}). Writing to {csv_filename}...")

        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Index", "Timestamp_US", "RSSI"])

            current = tail
            written = 0
            
            # Loop from tail to head (excluding head)
            while current != head:
                t = ts_arr[current]
                # Only write existing data (non-zero timestamp usually good indicator)
                if t != 0:
                    writer.writerow([written, t, rssi_arr[current]])
                    written += 1
                
                current = (current + 1) % RSSI_BUF_SIZE
        
        return csv_filename

    except Exception as e:
        print(f"  Slot {slot_index}: Parse error ({e})")
        return None

def main():
    parser = argparse.ArgumentParser(description="Extract and parse multi-slot buffer dumps")
    parser.add_argument("--port", "-p", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", "-b", default=460800, type=int, help="Baud rate")
    parser.add_argument("--file", "-f", help="Existing binary file to parse (skip extraction)")
    parser.add_argument("--output", "-o", help="Output bin filename", default="dumps.bin")
    
    args = parser.parse_args()
    
    bin_filename = args.output
    if args.file:
        bin_filename = args.file
    else:
        run_esptool_read(args.port, args.baud, bin_filename)

    file_size = os.path.getsize(bin_filename)
    num_slots = file_size // SLOT_SIZE
    
    print(f"Scanning {bin_filename} ({file_size} bytes, {num_slots} slots)...")
    
    found_count = 0
    with open(bin_filename, "rb") as f:
        for i in range(num_slots):
            offset = i * SLOT_SIZE
            f.seek(offset)
            chunk = f.read(SLOT_SIZE)
            if parse_slot(i, chunk):
                found_count += 1
    
    print(f"Done. Extracted {found_count} dumps.")

if __name__ == "__main__":
    main()
