import timeit
import simulate_rotation
import numpy as np
import random

# Setup data
window_size = 1000

# 1. Setup for NUFFT (1000 raw samples)
# Generate random timestamps (approx 2ms intervals for 500Hz) and RSSI
start_time = 1000000
timestamps = [start_time + i * 2000 + random.randint(-100, 100) for i in range(window_size)]
rssi = [random.randint(-90, -30) for _ in range(window_size)]
state_nufft = simulate_rotation.RotationState()

# 2. Setup for Autocorrelation (1000 interpolated samples)
# run_rotation_task expects `head` index. `data` needs to be at least head size.
# CORR_WINDOW is used inside. We need to patch it.
simulate_rotation.CORR_WINDOW = 1000
# It accesses data[head-window : head] and data[head-window-lag : head-lag]
# Max lag is END_LAG (1000).
# So we need at least CORR_WINDOW + END_LAG samples.
# But "Window size of 1000" usually implies the correlation window length.
# The user asked "Autocorrelation with a window size of 1000".
# This corresponds to CORR_WINDOW = 1000.
# We need to ensure the buffer is large enough.
buffer_len = 3000
interp_rssi = [random.randint(-90, -30) for _ in range(buffer_len)]
head = 2500 # Safe index
state_autocorr = simulate_rotation.RotationState()

def bench_nufft():
    simulate_rotation.run_rotation_task_nufft(timestamps, rssi, state_nufft)

def bench_autocorr():
    # Reset state if needed, but for timing it shouldn't matter much 
    # unless it triggers different branches.
    # The code has a coarse search (always runs) and a fine search (runs if valid).
    # To be fair, let's try to make it find something or not?
    # Random noise likely won't find a valid lock, so it might skip fine search.
    # User probably wants "Worst case" or "Typical case"?
    # If I give it a sine wave it effectively finds it.
    simulate_rotation.run_rotation_task(interp_rssi, head, state_autocorr)

# Generate a sine wave to ensure Autocorr does the full work (fine search phase)
# 20ms period = 50Hz. 100us intervals. Period = 200 samples.
# Lags 200..1000. 200 is included.
import math
for i in range(buffer_len):
    interp_rssi[i] = int(-60 + 30 * math.sin(2 * math.pi * i / 200.0))

# Same for NUFFT
for i in range(window_size):
    t_sec = (timestamps[i] - start_time) / 1e6
    rssi[i] = int(-60 + 30 * math.sin(2 * math.pi * 50 * t_sec))

print(f"Benchmarking with Window Size = {window_size}...")
print(f"NUFFT input: {len(timestamps)} raw samples")
print(f"Autocorr input: CORR_WINDOW={simulate_rotation.CORR_WINDOW} (interpolated samples)")

# Run once to verify no errors
bench_nufft()
bench_autocorr()

# Time it
n_loops = 100
t_nufft = timeit.timeit(bench_nufft, number=n_loops)
t_autocorr = timeit.timeit(bench_autocorr, number=n_loops)

print(f"\nResults ({n_loops} loops):")
print(f"NUFFT Total Time: {t_nufft:.4f} s")
print(f"NUFFT Avg Time:   {t_nufft/n_loops*1000:.4f} ms")
print(f"Autocorr Total Time: {t_autocorr:.4f} s")
print(f"Autocorr Avg Time:   {t_autocorr/n_loops*1000:.4f} ms")

ratio = t_nufft / t_autocorr
print(f"\nRatio (NUFFT / Autocorr): {ratio:.2f}x")
if ratio < 1:
    print(f"NUFFT is {1/ratio:.2f}x faster")
else:
    print(f"Autocorrelation is {ratio:.2f}x faster")
