#!/usr/bin/env python3
import sys
import struct
import csv
import argparse
import os
import subprocess
from datetime import datetime

import json

# Configuration matching Receiver Firmware
RSSI_BUF_SIZE = 6000
SLOT_SIZE = 0x20000     # 128KB
PARTITION_SIZE = 0x200000 # 2MB
PARTITION_OFFSET = 0x187000

# C Struct Layout
# typedef struct {
#   int8_t rssi[RSSI_BUF_SIZE];              // 6000 bytes
#   int64_t timestamp[RSSI_BUF_SIZE];        // 6000 * 8 = 48000 bytes
#   control_packet_t control[RSSI_BUF_SIZE]; // 6000 * 12 = 72000 bytes
#                                            // (packed: u8(1) + magic(1) + u16(2) + f(4) + f(4) = 12 bytes)
#   int head;                                // 4 bytes
#   int tail;                                // 4 bytes
#   int64_t last_timestamp;                  // 8 bytes
#   int8_t last_rssi;                        // 1 byte
#   app_config_packet_t config;              // 28 bytes
# } control_circular_buffer_t;

# app_config_packet_t (packed):
# uint8_t type; (1)
# uint8_t magic; (1)
# uint8_t dshot_pin_a; (1)
# uint8_t dshot_pin_b; (1)
# uint8_t led_pin; (1)
# uint8_t rotation_source; (0=CSI, 1=ESPNOW, 2=CSI_DR, 3=ESPNOW_DR)
# uint16_t step_lag; (2)
# uint16_t step_window; (2)
# float throttle_multiplier; (4)
# float translation_multiplier; (4)
# uint16_t correlation_window; (2)
# uint16_t smoothing_window; (2)
# float phase_offset; (4)
# uint8_t translation_method; (1)
# uint8_t led_display_mode; (1)
CONFIG_SIZE = 28
CONFIG_FMT = "<BBBBBBHHffHHfBB"

CONTROL_PKT_SIZE = 12 # 1 + 1 + 2 + 4 + 4
CONTROL_ARR_SIZE = RSSI_BUF_SIZE * CONTROL_PKT_SIZE

STRUCT_FMT = f"<{RSSI_BUF_SIZE}b{RSSI_BUF_SIZE}q{CONTROL_ARR_SIZE}siiqb{CONFIG_SIZE}s"
EXPECTED_SIZE = struct.calcsize(STRUCT_FMT)

# Control Packet Unpacker
# uint8_t type; uint8_t magic; uint16_t throttle; float vector_x; float vector_y;
CONTROL_PKT_FMT = "<BBHff"

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
        print(f"  Slot {slot_index}: Data too short ({len(data)} < {EXPECTED_SIZE})")
        return None

    try:
        unpacked = struct.unpack(STRUCT_FMT, data[:EXPECTED_SIZE])
        
        rssi_arr = unpacked[0 : RSSI_BUF_SIZE]
        ts_arr = unpacked[RSSI_BUF_SIZE : 2 * RSSI_BUF_SIZE]
        control_bytes = unpacked[2 * RSSI_BUF_SIZE] # This is a bytestring
        
        head = unpacked[2 * RSSI_BUF_SIZE + 1]
        tail = unpacked[2 * RSSI_BUF_SIZE + 2]
        last_ts = unpacked[2 * RSSI_BUF_SIZE + 3]
        # last_rssi
        config_bytes = unpacked[2 * RSSI_BUF_SIZE + 5]

        # Basic validation
        if head < -1 or head >= RSSI_BUF_SIZE:
            return None
        if tail < -1 or tail >= RSSI_BUF_SIZE:
            return None
        
        # If head == -1, buffer is logically empty/uninitialized for our purposes
        if head == -1:
            return None

        # Parse Config
        cfg_unpacked = struct.unpack(CONFIG_FMT, config_bytes)
        config_dict = {
            "dshot_pin_a": cfg_unpacked[2],
            "dshot_pin_b": cfg_unpacked[3],
            "led_pin": cfg_unpacked[4],
            "rotation_source": cfg_unpacked[5],
            "step_lag": cfg_unpacked[6],
            "step_window": cfg_unpacked[7],
            "throttle_multiplier": cfg_unpacked[8],
            "translation_multiplier": cfg_unpacked[9],
            "correlation_window": cfg_unpacked[10],
            "smoothing_window": cfg_unpacked[11],
            "phase_offset": cfg_unpacked[12],
            "translation_method": cfg_unpacked[13],
            "led_display_mode": cfg_unpacked[14]
        }

        # Save Config JSON
        json_filename = f"dump_slot{slot_index:02d}.json"
        with open(json_filename, 'w') as f:
            json.dump(config_dict, f, indent=4)

        csv_filename = f"dump_slot{slot_index:02d}.csv"
        print(f"  Slot {slot_index}: Found valid dump (Head {head}, Tail {tail}). Writing to {csv_filename} and {json_filename}...")

        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Index", "Timestamp_US", "RSSI", "Throttle", "VectorX", "VectorY"])

            current = tail
            written = 0
            
            # Loop from tail to head (excluding head)
            while current != head:
                t = ts_arr[current]
                # Only write existing data (non-zero timestamp usually good indicator)
                if t != 0:
                    # Parse Control Packet for this index
                    c_offset = current * CONTROL_PKT_SIZE
                    c_data = control_bytes[c_offset : c_offset + CONTROL_PKT_SIZE]
                    c_pkt = struct.unpack(CONTROL_PKT_FMT, c_data)
                    # c_pkt = (type, magic, throttle, vx, vy)
                    
                    writer.writerow([written, t, rssi_arr[current], c_pkt[2], c_pkt[3], c_pkt[4]])
                    written += 1
                
                current = (current + 1) % RSSI_BUF_SIZE
        
        return csv_filename

    except Exception as e:
        print(f"  Slot {slot_index}: Parse error ({e})")
        import traceback
        traceback.print_exc()
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
            # Pass a larger chunk if possible, or just SLOT_SIZE.
            # Safety: Ensure chunk is at least EXPECTED_SIZE
            if len(chunk) >= EXPECTED_SIZE:
                if parse_slot(i, chunk):
                    found_count += 1
            else:
                 print(f"  Slot {i}: Chunk size {len(chunk)} < Expected {EXPECTED_SIZE}")
    
    print(f"Done. Extracted {found_count} dumps.")

if __name__ == "__main__":
    main()
