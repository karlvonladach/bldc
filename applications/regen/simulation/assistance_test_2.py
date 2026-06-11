import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#df = pd.read_excel('livetest260601.xlsx', sheet_name='2026-06-01_11-54-25', nrows=1528)
df = pd.read_excel('livetest260603.xlsx', sheet_name='data', nrows=7589)

# Convert all columns to numeric, coerce errors to NaN
df = df.apply(pd.to_numeric, errors='coerce')

# Drop rows with NaN values
# df = df.dropna()

# Store each column into separate arrays
# Assuming no header in the data range, columns are 0-indexed
time          = col_A = df.iloc[:, 0].values
pedal_rpm     = col_B = df.iloc[:, 1].values
pedal_torque  = col_C = df.iloc[:, 2].values
motor_rpm     = Col_D = df.iloc[:, 3].values
motor_current = Col_E = df.iloc[:, 4].values
speed         = Col_F = df.iloc[:, 5].values
altitude      = Col_G = df.iloc[:, 6].values
expacc_realacc= Col_H = df.iloc[:, 7].values

time2 = (time - time[0]) / 1000.0

valid = (
	np.isfinite(time2)
	& np.isfinite(pedal_rpm)
	& np.isfinite(pedal_torque)
	& np.isfinite(motor_rpm)
	& np.isfinite(motor_current)
    & np.isfinite(speed)
    & np.isfinite(altitude)
    & np.isfinite(expacc_realacc)
)
time2 = time2[valid]
pedal_rpm = pedal_rpm[valid]
pedal_torque = pedal_torque[valid]
motor_rpm = motor_rpm[valid]
motor_current = motor_current[valid]
speed = speed[valid]
altitude = altitude[valid]
expacc_realacc = expacc_realacc[valid]

# Human and motor power.
pedal_omega = pedal_rpm * (2.0 * np.pi / 60.0)
motor_omega = motor_rpm * (2.0 * np.pi / 60.0)
human_power_w = pedal_torque * pedal_omega

# Approximate motor mechanical power using motor current and the torque constant
# used elsewhere in this simulation folder.
Kt = 0.62  # Nm/A
motor_torque_nm = motor_current * Kt
motor_power_w = motor_torque_nm * motor_omega

wheel_radius_m = 0.319
mass_kg = 100.0
wheel_velocity_m_s = motor_omega * wheel_radius_m
total_power_w = human_power_w + motor_power_w
expected_accel_m_s2 = np.divide(
	total_power_w,
	mass_kg * wheel_velocity_m_s,
	out=np.zeros_like(total_power_w),
	where=np.abs(wheel_velocity_m_s) > 1e-3,
)
expected_accel_m_s2 = np.clip(expected_accel_m_s2, 0.0, 50)

# Motor acceleration from RPM over time (RPM/s).
motor_accel_rpm_s = np.gradient(motor_rpm, time2)
motor_accel_rpm_s = np.clip(motor_accel_rpm_s, 0.0, None)

# Basic noise filtering: causal moving average (uses current and past samples only).
accel_filter_window = 9
n_samples = len(motor_accel_rpm_s)
trailing_sum = np.convolve(
	motor_accel_rpm_s,
	np.ones(accel_filter_window),
	mode='full',
)[:n_samples]
trailing_count = np.minimum(np.arange(1, n_samples + 1), accel_filter_window)
motor_accel_filtered_rpm_s = trailing_sum / trailing_count
motor_accel_filtered_rpm_s = np.clip(motor_accel_filtered_rpm_s, 0.0, 50)
motor_accel_m_s2 = motor_accel_rpm_s * (2.0 * np.pi / 60.0) * wheel_radius_m
motor_accel_filtered_m_s2 = motor_accel_filtered_rpm_s * (2.0 * np.pi / 60.0) * wheel_radius_m

# Bike acceleration based on gnss (m/s2).
accel_m_s2 = np.gradient(speed, time2)
accel_m_s2 = np.clip(accel_m_s2, 0.0, None)

# Create plot
fig, ax_signals = plt.subplots(figsize=(11, 7))
ax_power = ax_signals.twinx()
ax_accel = ax_signals.twinx()
ax_accel.spines['right'].set_position(('axes', 1.12))
ax_accel.set_frame_on(True)
ax_accel.patch.set_visible(False)

ax_signals.plot(time2, pedal_rpm, label='Pedal RPM', marker='o', markersize=2, linestyle='-', color='tab:blue')
ax_signals.plot(time2, motor_rpm, label='Motor RPM', marker='.', markersize=2, linestyle='-', color='tab:orange')
ax_signals.plot(time2, pedal_torque, label='Pedal Torque', linestyle='--', color='tab:green')
ax_signals.plot(time2, motor_current, label='Motor Current', linestyle='--', color='tab:purple')
ax_signals.plot(time2, (altitude-140)*10, label='Altitude', linestyle='-', color='#000000', alpha=0.8)
ax_signals.set_xlabel('time (s)')
ax_signals.set_ylabel('RPM / Torque / Current')
ax_signals.grid(True, alpha=0.3)

ax_power.plot(time2, human_power_w, label='Human Power (W)', linewidth=2.0, color='tab:red')
ax_power.plot(time2, motor_power_w, label='Motor Power (W)', linewidth=2.0, color='tab:brown')
ax_power.set_ylabel('Power (W)')

ax_accel.plot(time2, motor_accel_m_s2, label='Motor Accel Raw (m/s²)', color='tab:gray', alpha=0.35)
ax_accel.plot(time2, motor_accel_filtered_m_s2, label='Motor Accel Filtered (m/s²)', color='tab:cyan', linewidth=2.0)
# ax_accel.plot(time2, accel_m_s2, label='Bike Accel GNSS (m/s²)', color='tab:pink', linewidth=2.0)
ax_accel.plot(time2, expected_accel_m_s2, label='Expected Accel (m/s²)', color='tab:olive', linewidth=2.0)
ax_accel.plot(time2, expected_accel_m_s2 - motor_accel_m_s2, label='ExpAcc-RealAcc calculated', color='#555555', linewidth=2.0)
ax_accel.plot(time2, expacc_realacc, label='ExpAcc-RealAcc measured', color='#222222', linewidth=2.0)
ax_accel.set_ylabel('Acceleration (m/s²)')

lines_signals, labels_signals = ax_signals.get_legend_handles_labels()
lines_power, labels_power = ax_power.get_legend_handles_labels()
lines_accel, labels_accel = ax_accel.get_legend_handles_labels()
ax_signals.legend(
	lines_signals + lines_power + lines_accel,
	labels_signals + labels_power + labels_accel,
	loc='best',
)

ax_signals.set_title('Signals, Power, and Motor Acceleration')

plt.tight_layout()

# Display the plot
plt.show()
