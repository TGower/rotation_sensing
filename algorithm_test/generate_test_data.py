import csv
import os

INPUT_CSV = "csi_log2_part_49.csv"
OUTPUT_HEADER = "main/test_data.h"

def generate_header():
    # Locate the CSV file - it might be in cleaned_runs or just locally if we copied it?
    # Based on previous steps, it is at /home/t/esp/rotation_sensing/receiver/cleaned_runs/csi_log2_part_49.csv
    # But for this script I will assume we run it from the algorithm_test directory and I'll point to the absolute path or copy it.
    # Let's use the absolute path we found earlier.
    csv_path = "/home/t/esp/rotation_sensing/receiver/cleaned_runs/csi_log2_part_49.csv"
    
    timestamps = []
    rssi_values = []

    print(f"Reading from {csv_path}...")
    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ts = int(row['local_timestamp'])
                    rssi = int(row['rssi'])
                    timestamps.append(ts)
                    rssi_values.append(rssi)
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"Error: Could not find {csv_path}")
        return

    print(f"Read {len(timestamps)} samples.")

    print(f"Writing to {OUTPUT_HEADER}...")
    with open(OUTPUT_HEADER, 'w') as f:
        f.write("#ifndef TEST_DATA_H\n")
        f.write("#define TEST_DATA_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"static const size_t test_data_len = {len(timestamps)};\n\n")
        
        f.write("static const int64_t test_timestamps[] = {\n")
        for i, ts in enumerate(timestamps):
            f.write(f"    {ts}LL,")
            if (i + 1) % 10 == 0:
                f.write("\n")
        f.write("\n};\n\n")

        f.write("static const int8_t test_rssi[] = {\n")
        for i, r in enumerate(rssi_values):
            f.write(f"    {r},")
            if (i + 1) % 20 == 0:
                f.write("\n")
        f.write("\n};\n\n")

        f.write("#endif // TEST_DATA_H\n")
    
    print("Done.")

if __name__ == "__main__":
    generate_header()
