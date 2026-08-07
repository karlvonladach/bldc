import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Plot regen simulation values from an Excel file.')
parser.add_argument(
	'filename',
	nargs='?',
	default='bq1-ma',
	help='Excel filename without extension (default: bq1-ma)'
)
args = parser.parse_args()

filename = args.filename
df = pd.read_excel(f'{filename}.xlsx', sheet_name='data')

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

time          	 = get_col_values(df, 'time', 				'ms_today', 		'A')
pedal_rpm     	 = get_col_values(df, 'pedal_rpm', 		    'accX',		 		'AP')
wheel_rpm    	 = get_col_values(df, 'wheel_rpm',   		'accY', 			'AQ')
pedal_torque     = get_col_values(df, 'pedal_torque',     	'encoder_position', 'T')
pedal_torque_raw = get_col_values(df, 'pedal_torque_raw', 	'gyroY', 			'AT')
motor_rpm     	 = get_col_values(df, 'motor_rpm', 		    'accZ',		 		'AR')
motor_current 	 = get_col_values(df, 'motor_current', 	    'current_motor', 	'H')
speed         	 = get_col_values(df, 'speed', 			    'gnss_gVel',		'AZ')
altitude      	 = get_col_values(df, 'altitude', 			'gnss_alt', 		'AY')
extra_res      	 = get_col_values(df, 'extra_res', 			'temp_mos_1',	 	'D')
acc	        	 = get_col_values(df, 'acc', 		    	'temp_mos_2', 		'E')
human_power      = get_col_values(df, 'human_power', 		'temp_mos_3',		'F')
normal_res     	 = get_col_values(df, 'normal_res',        	'roll', 	    	'AM')
acc_filt     	 = get_col_values(df, 'acc_filt',        	'pitch', 	    	'AN')
astgain     	 = get_col_values(df, 'astgain',        	'yaw',			 	'AO')

time2 = (time - time[0]) / 1000.0

Kt = 0.62  # Nm/A
wheel_radius = 0.319
mass = 100.0
motor_gear_eff = 0.90
pedal_gear_eff = 0.95

# cf = 1Hz
BIQUAD_FILTER_B0 = 1.34910548
BIQUAD_FILTER_B1 = 0.0
BIQUAD_FILTER_B2 = -1.34910548
BIQUAD_FILTER_A1 = -1.14298050
BIQUAD_FILTER_A2 = 0.41280160

# cf = 0.5Hz
BTW_FILTER_05_B0 = 0.02008337
BTW_FILTER_05_B1 = 0.04016673	
BTW_FILTER_05_B2 = 0.02008337	
BTW_FILTER_05_A1 = -1.56101808	
BTW_FILTER_05_A2 = 0.64135154	

# cf = 1Hz
BTW_FILTER_1_B0 = 0.06745527
BTW_FILTER_1_B1 = 0.13491055
BTW_FILTER_1_B2 = 0.06745527
BTW_FILTER_1_A1 = -1.14298050
BTW_FILTER_1_A2 = 0.41280160

def apply_biquad_filter(x, b0, b1, b2, a1, a2):
	y = np.zeros_like(x, dtype=float)
	x1 = 0.0
	x2 = 0.0
	y1 = 0.0
	y2 = 0.0

	for i, xn in enumerate(x):
		yn = b0 * xn + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
		y[i] = yn
		x2, x1 = x1, xn
		y2, y1 = y1, yn

	return y

def safe_divide(numerator, denominator, min_abs=1e-6):
	numerator = np.asarray(numerator, dtype=float)
	denominator = np.asarray(denominator, dtype=float)
	out = np.full_like(numerator, np.nan, dtype=float)
	valid = np.isfinite(numerator) & np.isfinite(denominator) & (np.abs(denominator) > min_abs)
	np.divide(numerator, denominator, out=out, where=valid)
	return out

motor_power = (motor_rpm * motor_current * Kt * 2 * np.pi) / 60.0 * motor_gear_eff
motor_force = motor_current * Kt * motor_gear_eff / wheel_radius
wheel_speed = (wheel_rpm * 2 * np.pi * wheel_radius) / 60.0

valid = (
	np.isfinite(time2)
	& np.isfinite(pedal_rpm)
	& np.isfinite(pedal_torque)
	& np.isfinite(pedal_torque_raw)
	& np.isfinite(motor_rpm)
	& np.isfinite(motor_current)
	& np.isfinite(wheel_rpm)
	& np.isfinite(normal_res)
	& np.isfinite(wheel_speed)
    & np.isfinite(speed)
    & np.isfinite(altitude)
	& np.isfinite(extra_res)
	& np.isfinite(acc)
    & np.isfinite(human_power)
    & np.isfinite(astgain)
)
time2 = time2[valid]
pedal_rpm = pedal_rpm[valid]
pedal_torque = pedal_torque[valid]
pedal_torque_raw = pedal_torque_raw[valid]
motor_rpm = motor_rpm[valid]
motor_current = motor_current[valid]
wheel_rpm = wheel_rpm[valid]
normal_res = normal_res[valid]
motor_force = motor_force[valid]
wheel_speed = wheel_speed[valid]
speed = speed[valid]
altitude = altitude[valid]
extra_res = extra_res[valid]
acc = acc[valid]
human_power = human_power[valid]
astgain = astgain[valid]

pedal_rpm_filtered = apply_biquad_filter(
	pedal_rpm,
	BTW_FILTER_1_B0,
	BTW_FILTER_1_B1,
	BTW_FILTER_1_B2,
	BTW_FILTER_1_A1,
	BTW_FILTER_1_A2,
)

human_power_calculated = pedal_torque * pedal_rpm_filtered * 2 * np.pi / 60.0 * pedal_gear_eff
human_force = safe_divide(human_power, wheel_speed)
human_force_calculated = safe_divide(human_power_calculated, wheel_speed)
total_force = human_force + motor_force
extra_res_calculated = total_force - acc * mass - normal_res

extra_res_filtered = apply_biquad_filter(
	extra_res_calculated,
	BTW_FILTER_05_B0,
	BTW_FILTER_05_B1,
	BTW_FILTER_05_B2,
	BTW_FILTER_05_A1,
	BTW_FILTER_05_A2,
)

print(f"Data loaded: {len(time2)} valid samples.")

# pedal rpm, wheel rpm, torque_filtered, motor current, human+motor power, acceleration, altitude, extra_res
plt.figure(figsize=(10, 6))
plt.plot(time2, pedal_rpm, label='Pedal RPM', linewidth=2, color='orange')
#plt.plot(time2, pedal_rpm_filtered, label='Pedal RPM (Biquad)', linewidth=2, color='orange', linestyle='dotted')
plt.plot(time2, wheel_rpm, label='Wheel RPM', linewidth=2, color='blue')
#plt.plot(time2, wheel_rpm_filtered, label='Wheel RPM (Biquad)', linewidth=2, color='navy', linestyle='dotted')
#plt.plot(time2, np.gradient(wheel_speed, time2)*mass, label='Acc Calculated x Mass', linewidth=2, color='blue', linestyle='dashed')
#plt.plot(time2, wheel_accel_filtered*mass, label='Acc Filtered x Mass', linewidth=2, color='blue', linestyle='dashdot')
plt.plot(time2, pedal_torque, label='Pedal Torque', linewidth=2, color='green')
plt.plot(time2, pedal_torque_raw, label='Pedal Torque Raw', linewidth=2, color='green', linestyle='dashed')
plt.plot(time2, motor_current, label='Motor Current', linewidth=2, color='red')
plt.plot(time2, human_force, label='Human Force', linewidth=2, color='purple')
#plt.plot(time2, human_force_calculated, label='Human Force (calculated)', linewidth=2, color='purple', linestyle='dotted')
plt.plot(time2, motor_force, label='Motor Force', linewidth=2, color='brown')
#plt.plot(time2, motor_power/wheel_speed, label='Motor Force 2', linewidth=2, color='brown', linestyle='dashed')
plt.plot(time2, total_force, label='Total Force', linewidth=2, color='magenta')
plt.plot(time2, acc, label='Acc x 100', linewidth=2, color='cyan')
#plt.plot(time2, acc*mass+normal_res, label='Acc x Mass + R_norm', linewidth=2, color='cyan', linestyle='dashed')
plt.plot(time2, (altitude-140)*10, label='Altitude', linewidth=2, color='black')
plt.plot(time2, -extra_res, label='-Extra Res', linewidth=2, color='pink')
#plt.plot(time2, -(extra_res_calculated), label='-Extra Res (calculated)', linewidth=2, color='pink', linestyle='dotted')
#plt.plot(time2, -extra_res_filtered, label='-Extra Res (Biquad)', linewidth=2, color='pink', linestyle='dashed')
plt.plot(time2, astgain*100, label='AST Gain', linewidth=2, color='grey')


plt.xlabel('Time (s)')
plt.ylabel('Values')
plt.title(filename)
plt.legend()
plt.grid(True)

# # Figure 2: signals with their gradients + gradient histograms
# pedal_rpm_grad = np.gradient(pedal_rpm, time2)
# speed_grad = np.gradient(speed, time2)
# pedal_torque_raw_grad = np.gradient(pedal_torque_raw, time2)
# acc_grad = np.gradient(acc, time2)

# fig2, axes = plt.subplots(4, 2, figsize=(14, 16), sharex='col')

# signal_rows = [
# 	('Pedal RPM', pedal_rpm, pedal_rpm_grad),
# 	('Speed', speed, speed_grad),
# 	('Pedal Torque Raw', pedal_torque_raw, pedal_torque_raw_grad),
# 	('Acceleration', acc, acc_grad),
# ]

# for row, (name, signal, grad) in enumerate(signal_rows):
# 	ax_sig = axes[row, 0]
# 	ax_hist = axes[row, 1]

# 	ax_sig.plot(time2, signal, label=name, linewidth=2)
# 	ax_sig.plot(time2, grad, label=f'{name} Gradient', linewidth=1.5, linestyle='dashed')
# 	ax_sig.set_ylabel(name)
# 	ax_sig.grid(True)
# 	ax_sig.legend()

# 	grad_finite = grad[np.isfinite(grad)]
# 	ax_hist.hist(grad_finite, bins=50, alpha=0.8)
# 	ax_hist.set_title(f'{name} Gradient Histogram')
# 	ax_hist.set_ylabel('Count')
# 	ax_hist.grid(True)

# axes[-1, 0].set_xlabel('Time (s)')
# axes[-1, 1].set_xlabel('Gradient Value')
# fig2.suptitle(f'{filename} - Signals and Gradients', fontsize=14)
# fig2.tight_layout(rect=[0, 0.03, 1, 0.98])

plt.show()
