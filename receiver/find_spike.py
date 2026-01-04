import pandas as pd
import numpy as np

# Load simulation script to use its functions
import simulate_harmonic_tuning
from simulate_harmonic_tuning import run_simulation

csv_path = "/home/t/esp/rotation_sensing/receiver/dump_slot01.csv"
window_size = 1000

print(f"Running simulation to find spike around 0.49s...")
results = run_simulation(csv_path, window_size)

times = np.array(results['time'])
rates = np.array(results['rate'])

# Focus on 0.45s to 0.55s
mask = (times >= 0.45) & (times <= 0.55)
t_window = times[mask]
r_window = rates[mask]

max_idx = np.argmax(r_window)
max_rate = r_window[max_idx]
spike_time = t_window[max_idx]

print(f"Max Rate in window: {max_rate:.2f} Hz at {spike_time:.4f} s")

# Print surrounding values
for t, r in zip(t_window, r_window):
    print(f"t={t:.4f}, r={r:.2f}")
