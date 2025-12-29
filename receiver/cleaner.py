import csv
import os
import shutil
import argparse

def main():
    parser = argparse.ArgumentParser(description="Split CSI log into runs based on <1ms delta")
    parser.add_argument("--file", default="csi_log.csv", help="Input CSV file")
    parser.add_argument("--output-dir", default="cleaned_runs", help="Output directory")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: {args.file} not found.")
        return

    # Prepare output directory
    if os.path.exists(args.output_dir):
        shutil.rmtree(args.output_dir)
    os.makedirs(args.output_dir)

    print(f"Processing {args.file}...")
    
    with open(args.file, 'r') as f:
        reader = csv.reader(f) # Use simple reader to keep header intact easily
        header = next(reader)
        
        # Find index of 'local_timestamp'
        try:
            ts_idx = header.index("local_timestamp")
        except ValueError:
            print("Error: 'local_timestamp' column not found.")
            return

        current_run = []
        last_ts = None
        run_count = 0
        
        for row in reader:
            try:
                ts = int(row[ts_idx])
            except ValueError:
                continue

            if last_ts is None:
                # First packet ever
                current_run.append(row)
            else:
                delta = ts - last_ts
                
                # Check for gap > 1ms (1000us)
                # Note: first packet of a run usually has a large delta from previous run
                if delta >= 1000: 
                    # End of previous run
                    save_run(args.output_dir, run_count, header, current_run)
                    if len(current_run) >= 10: # Only count valid runs
                        run_count += 1
                    
                    # Start new run
                    current_run = [row]
                else:
                    # Continue current run
                    current_run.append(row)
            
            last_ts = ts
        
        # Save last run
        if current_run:
            save_run(args.output_dir, run_count, header, current_run)
            run_count += 1

    print(f"Done. Extracted {run_count} runs to '{args.output_dir}/'.")

def save_run(out_dir, run_idx, header, rows):
    # Filter small noise runs (e.g. single packets)
    if len(rows) < 10:
        return

    filename = os.path.join(out_dir, f"run_{run_idx:04d}.csv")
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

if __name__ == "__main__":
    main()
