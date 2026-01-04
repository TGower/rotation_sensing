import csv

filename = 'dump_slot00.csv'
timestamps = []
with open(filename, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        timestamps.append(int(row['Timestamp_US']))

print(f"Total samples: {len(timestamps)}")
backtracks = 0
for i in range(1, len(timestamps)):
    if timestamps[i] < timestamps[i-1]:
        print(f"Backtrack at index {i}: {timestamps[i-1]} -> {timestamps[i]}")
        backtracks += 1
        if backtracks > 5:
            print("...")
            break

if backtracks == 0:
    print("Timestamps are monotonic.")
