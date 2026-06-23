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
astgain     	 = get_col_values(df, 'astgain',        	'yaw',			 	'AO')

time2 = (time - time[0]) / 1000.0

Kt = 0.62  # Nm/A
wheel_radius = 0.319
mass = 100.0

motor_power = (motor_rpm * motor_current * Kt * 2 * np.pi) / 60.0
wheel_speed = (wheel_rpm * 2 * np.pi * wheel_radius) / 60.0

valid = (
	np.isfinite(time2)
	& np.isfinite(pedal_rpm)
	& np.isfinite(pedal_torque)
	& np.isfinite(pedal_torque_raw)
	& np.isfinite(motor_rpm)
	& np.isfinite(motor_current)
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
speed = speed[valid]
altitude = altitude[valid]
extra_res = extra_res[valid]
acc = acc[valid]
human_power = human_power[valid]
astgain = astgain[valid]

print(f"Data loaded: {len(time2)} valid samples.")

# pedal rpm, wheel rpm, torque_filtered, motor current, human+motor power, acceleration, altitude, extra_res
plt.figure(figsize=(10, 6))
plt.plot(time2, pedal_rpm, label='Pedal RPM', linewidth=2, color='orange')
plt.plot(time2, wheel_rpm, label='Wheel RPM', linewidth=2, color='blue')
plt.plot(time2, np.gradient(wheel_speed, time2)*mass, label='Acc Calculated x Mass', linewidth=2, color='blue', linestyle='dashed')
plt.plot(time2, pedal_torque, label='Pedal Torque', linewidth=2, color='green')
plt.plot(time2, pedal_torque_raw, label='Pedal Torque Raw', linewidth=2, color='green', linestyle='dashed')
plt.plot(time2, motor_current, label='Motor Current', linewidth=2, color='red')
#plt.plot(time2, human_power, label='Human Power', linewidth=2, color='purple')
#plt.plot(time2, motor_power, label='Motor Power', linewidth=2, color='brown')
plt.plot(time2, (human_power+motor_power)/wheel_speed, label='Total Force', linewidth=2, color='magenta')
plt.plot(time2, acc*mass, label='Acc x Mass', linewidth=2, color='cyan')
plt.plot(time2, acc*mass+normal_res, label='Acc x Mass + R_norm', linewidth=2, color='cyan', linestyle='dashed')
plt.plot(time2, (altitude-140)*10, label='Altitude', linewidth=2, color='black')
plt.plot(time2, extra_res, label='Extra Res', linewidth=2, color='pink')
#plt.plot(time2, astgain, label='AST Gain', linewidth=2, color='yellow')

plt.xlabel('Time (s)')
plt.ylabel('Values')
plt.title(filename)
plt.legend()
plt.grid(True)
plt.show()
