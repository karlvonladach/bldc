import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#df = pd.read_excel('livetest260601.xlsx', sheet_name='2026-06-01_11-54-25', nrows=1528)
#df = pd.read_excel('2026-06-08_00-44-10.xlsx', sheet_name='data')
#df = pd.read_excel('2026-06-07_19-07-44.xlsx', sheet_name='data')
df = pd.read_excel('2026-06-07_11-54-58.xlsx', sheet_name='data')
#df = pd.read_excel('2026-06-07_10-54-13.xlsx', sheet_name='data')

# Convert all columns to numeric, coerce errors to NaN
df = df.apply(pd.to_numeric, errors='coerce')

# Drop rows with NaN values
# df = df.dropna()

# Store each column into separate arrays using column names instead of fixed indices.
def get_col_values(frame, *aliases):
	for name in aliases:
		if name in frame.columns:
			return frame[name].values
	raise KeyError(f"None of these columns were found: {aliases}")


time          	 = Col_A  = get_col_values(df, 'time', 				'ms_today', 		'A')
pedal_rpm     	 = Col_AP = get_col_values(df, 'pedal_rpm', 		'accX',		 		'AP')
pedal_torque  	 = Col_AQ = get_col_values(df, 'pedal_torque', 		'accY', 			'AQ')
pedal_torque_raw = Col_AT = get_col_values(df, 'pedal_torque_raw', 	'gyroY', 			'AT')
motor_rpm     	 = Col_AR = get_col_values(df, 'motor_rpm', 		'accZ',		 		'AR')
motor_current 	 = Col_H  = get_col_values(df, 'motor_current', 	'current_motor', 	'H')
speed         	 = Col_AZ = get_col_values(df, 'speed', 			'gnss_gVel',		'AZ')
altitude      	 = Col_AY = get_col_values(df, 'altitude', 			'gnss_alt', 		'AY')
expacc        	 = Col_D  = get_col_values(df, 'expacc', 			'temp_mos_1',	 	'D')
realacc	      	 = Col_E  = get_col_values(df, 'realacc', 			'temp_mos_2', 		'E')
expacc_realacc	 = Col_AO = get_col_values(df, 'expacc_realacc', 	'yaw',			 	'AO')

time2 = (time - time[0]) / 1000.0

valid = (
	np.isfinite(time2)
	& np.isfinite(pedal_rpm)
	& np.isfinite(pedal_torque)
	& np.isfinite(pedal_torque_raw)
	& np.isfinite(motor_rpm)
	& np.isfinite(motor_current)
    & np.isfinite(speed)
    & np.isfinite(altitude)
	& np.isfinite(expacc)
	& np.isfinite(realacc)
    & np.isfinite(expacc_realacc)
)
time2 = time2[valid]
pedal_rpm = pedal_rpm[valid]
pedal_torque = pedal_torque[valid]
pedal_torque_raw = pedal_torque_raw[valid]
motor_rpm = motor_rpm[valid]
motor_current = motor_current[valid]
speed = speed[valid]
altitude = altitude[valid]
expacc = expacc[valid]
realacc = realacc[valid]
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

# Apply similar causal moving average filtering to real acceleration.
realacc_trailing_sum = np.convolve(
	realacc,
	np.ones(accel_filter_window),
	mode='full',
)[:n_samples]
realacc_trailing_count = np.minimum(np.arange(1, n_samples + 1), accel_filter_window)
realacc_filtered = realacc_trailing_sum / realacc_trailing_count

accel_filter_window = 101
# Apply similar causal moving average filtering to expacc_realacc difference.
expacc_realacc_trailing_sum = np.convolve(
	expacc_realacc,
	np.ones(accel_filter_window),
	mode='full',
)[:n_samples]
expacc_realacc_trailing_count = np.minimum(np.arange(1, n_samples + 1), accel_filter_window)
expacc_realacc_filtered = expacc_realacc_trailing_sum / expacc_realacc_trailing_count

grad_m_s = np.gradient(altitude, time2)*500
grad_m_s_trailing_sum = np.convolve(
	grad_m_s,
	np.ones(accel_filter_window),
	mode='full',
)[:n_samples]
grad_m_s_trailing_count = np.minimum(np.arange(1, n_samples + 1), accel_filter_window)
grad_m_s_filtered = grad_m_s_trailing_sum / grad_m_s_trailing_count


# Bike acceleration based on gnss (m/s2).
accel_m_s2 = np.gradient(speed, time2)
accel_m_s2 = np.clip(accel_m_s2, 0.0, None)


def get_zero_fraction(values, default=0.5):
	finite_values = values[np.isfinite(values)]
	if finite_values.size == 0:
		return default
	data_min = float(np.min(finite_values))
	data_max = float(np.max(finite_values))
	if data_min < 0.0 < data_max:
		return -data_min / (data_max - data_min)
	return default


def set_ylim_with_shared_zero(axis, values, zero_fraction, pad_fraction=0.08):
	finite_values = values[np.isfinite(values)]
	if finite_values.size == 0:
		return
	data_min = float(np.min(finite_values))
	data_max = float(np.max(finite_values))
	if data_min == data_max:
		span = max(abs(data_max), 1.0)
	else:
		span = data_max - data_min

	span *= 1.0 + pad_fraction
	if zero_fraction <= 0.0:
		zero_fraction = 0.5
	if zero_fraction >= 1.0:
		zero_fraction = 0.5

	span = max(
		span,
		data_max / max(1.0 - zero_fraction, 1e-9) if data_max > 0.0 else 0.0,
		-data_min / max(zero_fraction, 1e-9) if data_min < 0.0 else 0.0,
	)
	axis.set_ylim(-zero_fraction * span, (1.0 - zero_fraction) * span)

# Create plot
fig, ax_signals = plt.subplots(figsize=(11, 7))
ax_power = ax_signals.twinx()
ax_accel = ax_signals.twinx()
ax_accel.spines['right'].set_position(('axes', 1.12))
ax_accel.set_frame_on(True)
ax_accel.patch.set_visible(False)

##ax_signals.plot(time2, pedal_rpm, label='Pedal RPM', marker='o', markersize=2, linestyle='-', color='tab:blue')
ax_signals.plot(time2, motor_rpm, label='Motor RPM', linestyle='-', color='tab:orange', alpha=0.2)
##ax_signals.plot(time2, pedal_torque, label='Pedal Torque', linestyle='--', color='tab:green')
##ax_signals.plot(time2, motor_current, label='Motor Current', linestyle='--', color='tab:purple')
ax_signals.plot(time2, (altitude-140)*10, label='Altitude', linestyle='-', color='#000000', alpha=0.8)
ax_signals.plot(time2, grad_m_s_filtered, label='Altitude Gradient', linestyle='-', color='tab:cyan', alpha=0.8)
ax_signals.set_xlabel('time (s)')
ax_signals.set_ylabel('RPM / Torque / Current')
ax_signals.grid(True, alpha=0.3)

##ax_power.plot(time2, human_power_w, label='Human Power (W)', linewidth=2.0, color='tab:red')
##ax_power.plot(time2, motor_power_w, label='Motor Power (W)', linewidth=2.0, color='tab:brown')
ax_power.set_ylabel('Power (W)')

##ax_accel.plot(time2, expacc, label='ExpAcc measured', color='tab:pink', linewidth=2.0)
##ax_accel.plot(time2, realacc_filtered, label='RealAcc measured', color='tab:cyan', linewidth=2.0)
ax_accel.plot(time2, expacc_realacc_filtered, label='ExpAcc-RealAcc measured', color='#222222', linewidth=2.0)
ax_accel.set_ylabel('Acceleration (m/s²)')

shared_zero_fraction = get_zero_fraction(np.concatenate([expacc, realacc, expacc_realacc]))
set_ylim_with_shared_zero(
	ax_signals,
	np.concatenate([pedal_rpm, motor_rpm, pedal_torque, motor_current, (altitude - 140.0) * 10.0]),
	shared_zero_fraction,
)
set_ylim_with_shared_zero(
	ax_power,
	np.concatenate([human_power_w, motor_power_w]),
	shared_zero_fraction,
)
set_ylim_with_shared_zero(
	ax_accel,
	np.concatenate([expacc, realacc, expacc_realacc]),
	shared_zero_fraction,
)

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
