import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

df = pd.read_excel('nyom_kiert.xlsx', sheet_name='1', nrows=10001)

# Convert all columns to numeric, coerce errors to NaN
df = df.apply(pd.to_numeric, errors='coerce')

# Drop rows with NaN values
# df = df.dropna()

# Store each column into separate arrays
# Assuming no header in the data range, columns are 0-indexed
time      = col_A = df.iloc[:, 0].values  # Column A (index 0)
pedal_rpm = col_B = df.iloc[:, 1].values  # Column B (index 1)
brake_pos = col_D = df.iloc[:, 3].values  # Column D (index 2)
hall1     = Col_N = df.iloc[:, 13].values / 20.0  # Column N (index 13)
hall2     = Col_P = df.iloc[:, 15].values / 20.0  # Column P (index 15)
torque    = Col_T = df.iloc[:, 19].values  # Column T (index 19)

# Rebuild time from first to last value with the same number of entries,
# then shift so it starts at 0.
time_interp = np.linspace(time[0], time[-1], num=len(time))
time2 = time_interp - time_interp[0]

# Quadrature decoding order for forward rotation:
# (0,0) -> (0,1) -> (1,1) -> (1,0) -> (0,0)
forward_edges = {
	((0, 0), (0, 1)),
	((0, 1), (1, 1)),
	((1, 1), (1, 0)),
	((1, 0), (0, 0)),
}
backward_edges = {(b, a) for (a, b) in forward_edges}

n_entries = min(10000, len(hall1), len(hall2), len(torque), len(time2))
torque_window = deque(maxlen=36)
torque_ma_filtered = np.zeros(n_entries)
last_filtered = 0.0

prev_state = (int(round(hall1[0])), int(round(hall2[0])))
prev_state = (1 if prev_state[0] else 0, 1 if prev_state[1] else 0)

for i in range(n_entries):
	curr_state = (int(round(hall1[i])), int(round(hall2[i])))
	curr_state = (1 if curr_state[0] else 0, 1 if curr_state[1] else 0)

	transition = (prev_state, curr_state)

	if transition in forward_edges:
		torque_window.append(float(torque[i]))
		last_filtered = float(np.mean(torque_window))*2
	elif transition in backward_edges:
		torque_window.clear()
		last_filtered = 0.0

	torque_ma_filtered[i] = last_filtered
	prev_state = curr_state

rpm_torque_product = pedal_rpm[:n_entries] * torque_ma_filtered / 100.0

# Create plot
plt.figure(figsize=(10, 6))
plt.plot(time2[:n_entries], pedal_rpm[:n_entries], label='Pedal RPM', marker='o', markersize=2, linestyle='-')
plt.plot(time2[:n_entries], torque[:n_entries], label='Torque', marker='s', markersize=2, linestyle='-')
plt.plot(time2[:n_entries], torque_ma_filtered, label='Torque MA Filtered', linewidth=2)
plt.plot(time2[:n_entries], rpm_torque_product, label='Pedal RPM * Torque MA Filtered', linewidth=2)

plt.xlabel('time')
plt.ylabel('Values')
plt.title('Cadence and Torque')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Display the plot
plt.show()
