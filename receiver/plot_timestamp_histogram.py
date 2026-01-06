import csv
import matplotlib.pyplot as plt
import sys
import os

def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'dump_slot13.csv'

    timestamps = []
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ts = int(row['Timestamp_US'])
                    if ts > 0:
                        timestamps.append(ts)
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
        return

    if not timestamps:
        print("No valid timestamps found.")
        return

    # Sort timestamps to ensure correct delta calculation (circular buffer dump might be ordered or not)
    # The load_raw_data in simulate_rotation sorts them, so we should too.
    timestamps.sort()

    deltas = []
    MAX_VAL = 5000
    for i in range(len(timestamps) - 1):
        d = timestamps[i+1] - timestamps[i]
        # Filter clearly invalid negative jumps 
        if d > 0:
            if d > MAX_VAL:
                 deltas.append(MAX_VAL + 25) # Place in the middle of the overflow bin
            else:
                 deltas.append(d)

    print(f"Calculated {len(deltas)} deltas.")
    
    # Bins: 0, 50, ..., 5000, 5050
    bins = list(range(0, MAX_VAL + 51, 50))
    
    plt.figure(figsize=(12, 6))
    n, bins_out, patches = plt.hist(deltas, bins=bins, color='skyblue', edgecolor='black')
    
    # Highlight the 'More' bin
    patches[-1].set_facecolor('salmon')
    
    plt.title(f'Histogram of Timestamp Deltas ({os.path.basename(filename)})')
    plt.xlabel('Delta (us)')
    plt.ylabel('Count')
    plt.grid(True, alpha=0.5)
    
    # Custom x-ticks to show "More"
    # Get current ticks
    ticks = list(plt.xticks()[0])
    # Ensure 5000 is there
    if 5000 not in ticks:
        ticks.append(5000)
    ticks.sort()
    
    # Filter ticks to reasonable range
    ticks = [t for t in ticks if t <= 5000]
    
    # Add the overflow tick
    ticks.append(5025)
    labels = [str(int(t)) for t in ticks[:-1]] + ["More"]
    
    plt.xticks(ticks, labels, rotation=45)
    
    # Log scale might be useful if there are outliers
    # plt.yscale('log')
    
    out_png = f"timestamp_histogram_{os.path.basename(filename)}.png"
    plt.savefig(out_png, dpi=300)
    print(f"Saved {out_png}")

if __name__ == "__main__":
    main()
