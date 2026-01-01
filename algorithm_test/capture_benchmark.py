import serial
import time
import sys
import argparse
import re
import os

def parse_log(filename):
    results = []
    bench_times = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            # RESULT: ts, rssi, lag, error
            if line.startswith("RESULT:"):
                parts = line.split(',')
                if len(parts) >= 2:
                    val = line.split('RESULT:')[1].strip()
                    results.append(val)
            
            # BENCH: lag, error, time_us, [opt_time_us]
            if line.startswith("BENCH:"):
                parts = line.replace("BENCH:", "").split(',')
                if len(parts) >= 3:
                    try:
                        time_us = int(parts[2].strip())
                        bench_times.append(time_us)
                    except ValueError:
                        pass
                        
    return results, bench_times

def compare_results(baseline_file, result_file):
    print(f"\n--- Comparing {result_file} vs {baseline_file} ---")
    
    if not os.path.exists(baseline_file):
        print(f"Baseline file {baseline_file} not found. Skipping comparison.")
        return

    base_res, base_times = parse_log(baseline_file)
    curr_res, curr_times = parse_log(result_file)
    
    # 1. Verification (Correctness)
    print(f"Comparing {len(curr_res)} results...")
    mismatches = 0
    for i, (b, c) in enumerate(zip(base_res, curr_res)):
        if b != c:
            print(f"Mismatch at sample {i}:")
            print(f"  Base: {b}")
            print(f"  Curr: {c}")
            mismatches += 1
            if mismatches > 5:
                print("... limiting mismatch output")
                break
    
    if mismatches == 0 and len(curr_res) == len(base_res) and len(curr_res) > 0:
        print("PASS: Results match baseline exactly.")
    elif len(curr_res) == 0:
        print("FAIL: No results found in current run.")
    else:
        print(f"FAIL: Found {mismatches} mismatches (or length diff: {len(base_res)} vs {len(curr_res)})")

    # 2. Performance (Timing)
    if base_times and curr_times:
        print(f"\nPerformance ({len(curr_times)} samples):")
        avg_base = sum(base_times) / len(base_times)
        avg_curr = sum(curr_times) / len(curr_times)
        
        diff = avg_base - avg_curr
        ratio = avg_base / avg_curr if avg_curr > 0 else 0.0
        
        print(f"  Avg Time Base: {avg_base:.2f} us")
        print(f"  Avg Time Curr: {avg_curr:.2f} us")
        print(f"  Improvement:   {diff:.2f} us ({ratio:.2f}x speedup)")
    else:
        print("Timing comparison skipped (missing data)")

def capture_benchmark(port, baudrate, output_file, baseline_file=None):
    print(f"Connecting to {port} at {baudrate}...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    print("waiting for benchmark start...")
    
    # Wait for start
    in_run = False
    f = open(output_file, 'w')
    
    try:
        while True:
            line_bytes = ser.readline()
            if not line_bytes:
                continue
                
            try:
                line = line_bytes.decode('utf-8', errors='ignore').strip()
            except:
                continue
                
            print(line) # Print to console so user sees progress

            if "--- Starting New Benchmark Run ---" in line:
                in_run = True
                print(f"Benchmark started! Capturing to {output_file}...")
                f.write(line + "\n")
                continue

            if in_run:
                # Capture everything during the run
                f.write(line + "\n")
                f.flush()

                if "Run Complete" in line:
                    print("Benchmark run complete.")
                    break
                    
                if "Limit of" in line and "reached" in line:
                    print("Limit reached.")
                    break
                    
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        f.close()
        ser.close()
        print(f"Log saved to {output_file}")
    
    if baseline_file:
        compare_results(baseline_file, output_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture algorithm benchmark output")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--out", default="benchmark_baseline.log", help="Output log file")
    parser.add_argument("--baseline", default="benchmark_baseline.log", help="Baseline log file for comparison")
    
    args = parser.parse_args()
    
    # Check if we are overwriting baseline, if so, don't compare against itself unless forced? 
    # Actually, comparing against itself is fine (should be 1x match). 
    # But if user wants to establish new baseline, they just run --out benchmark_baseline.log
    
    compare_baseline = args.baseline
    if args.out == args.baseline and not os.path.exists(args.baseline):
        compare_baseline = None # Creating for first time

    capture_benchmark(args.port, args.baud, args.out, compare_baseline)
