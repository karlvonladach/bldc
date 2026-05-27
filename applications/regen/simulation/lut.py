import numpy as np
import matplotlib.pyplot as plt
import argparse

# Dimension of the LUT (e.g., 16x16 grid)
torque_steps = 16
cadence_steps = 16

# Input ranges
torque_axis = np.linspace(0, 80, torque_steps)     # 0 to 80 Nm
cadence_axis = np.linspace(0, 120, cadence_steps)  # 0 to 120 RPM

parser = argparse.ArgumentParser(description="Generate assist LUT from cadence and torque.")
parser.add_argument(
    "--assist-level",
    default="Turbo",
    choices=["Eco", "Tour", "Sport", "Turbo"],
    help="Assist level to generate LUT for.",
)
args = parser.parse_args()
assist_level = args.assist_level

# Parameters
if assist_level == "Eco":
    K_base = 0.3        # 30% baseline assist
    K_max = 0.6         # 60% maximum assist
    alpha = 1.0         # Linear
elif assist_level == "Tour":
    K_base = 0.6        # 80% baseline assist
    K_max = 1.2         # 240% maximum assist
    alpha = 1.2         # Slightly progressive
elif assist_level == "Sport":
    K_base = 0.8        # 80% baseline assist
    K_max = 2.0         # 200% maximum assist
    alpha = 1.5         # More progressive
else:  # "Turbo"
    K_base = 1.2        # 100% baseline assist
    K_max = 3.0         # 320% maximum assist
    alpha = 1.3         # Progressive

P_max_motor = 1000.0 # Legal continuous watt limit

rpm_start = 15.0       # Center of the low-end turn-on ramp (RPM)
beta_1 = 0.3           # Steepness of low-end ramp (higher = sharper turn on)
rpm_cutoff = 105.0     # Center of the high-end decay ramp (RPM)
beta_2 = 0.2           # Steepness of high-end fade out

def calculate_cadence_shaping(rpm):
    """
    Computes the asymmetrical bell-curve shaping factor using two sigmoids.
    Returns a coefficient between 0.0 and 1.0.
    """
    # Low-end ramp up (Sigmoid 1)
    # Avoid division by zero by protecting exponent boundaries
    low_ramp = 1.0 / (1.0 + np.exp(-beta_1 * (rpm - rpm_start)))
    
    # High-end fade out (Sigmoid 2)
    high_decay = 1.0 - (1.0 / (1.0 + np.exp(-beta_2 * (rpm - rpm_cutoff))))
    
    return low_ramp * high_decay

def to_c_array_2d(name, arr):
    rows, cols = arr.shape
    lines = [f"static const float {name}[{rows}][{cols}] = {{"]
    for row in arr:
        values = ", ".join(f"{value:.6f}f" for value in row)
        lines.append(f"    {{{values}}},")
    lines.append("};")
    return "\n".join(lines)

def to_c_array_2d_u32(name, arr):
    rows, cols = arr.shape
    lines = [f"static const uint32_t {name}[{rows}][{cols}] = {{"]
    for row in arr:
        values = ", ".join(f"{int(value)}u" for value in row)
        lines.append(f"    {{{values}}},")
    lines.append("};")
    return "\n".join(lines)

def to_python_array_2d(name, arr):
    rows = []
    for row in arr:
        rows.append("[" + ", ".join(f"{value:.6f}" for value in row) + "]")
    return f"{name} = [\n    " + ",\n    ".join(rows) + "\n]"

lut = np.zeros((cadence_steps, torque_steps))

for i, rpm in enumerate(cadence_axis):
    for j, tau in enumerate(torque_axis):
        # Calculate raw human mechanical power
        omega = rpm * (2 * np.pi / 60.0)
        p_human = tau * omega
        
        # 1. Non-linear torque scaling factor
        tau_ratio = min(tau / 60.0, 1.0) # Normalise to max active scaling torque
        k_tau = K_base + (K_max - K_base) * (tau_ratio ** alpha)
        
        # 2. Cadence shaping factor (taper off below 15 RPM and above 100 RPM)
        f_cadence = calculate_cadence_shaping(rpm)
            
        # Combine elements
        p_target = p_human * k_tau * f_cadence
        
        # Enforce hard clipping limit
        lut[i, j] = min(p_target, P_max_motor)

# 'lut' array is now ready to be exported as a C-array static const float LUT[16][16]

# Derived arrays: cadence*torque and normalized motor power
cadence_torque_product = np.outer(cadence_axis*2*np.pi/60, torque_axis)
power_over_cadence_torque = np.divide(
    lut,
    cadence_torque_product,
    out=np.full_like(lut, np.nan),
    where=cadence_torque_product != 0,
)

# Motor current approximation (assuming 1:2 pedal to wheel gear ratio and 1:31 wheel to motor gear ratio)
gear_ratio = 62
Kt = 0.08  # Nm/A, motor torque constant
motor_current = np.divide(
    lut, 
    cadence_axis[:, None]*2*np.pi/60*gear_ratio*Kt, 
    out=np.full_like(lut, np.nan), 
    where=cadence_axis[:, None] != 0,
)
motor_current_relative = np.divide(
    motor_current,
    60.0,  # Assume 60A max current for normalization
    out=np.full_like(motor_current, np.nan),
    where=60.0 != 0,
)

motor_current_relative_export = np.nan_to_num(motor_current_relative, nan=0.0) * 65536
motor_current_relative_export = np.clip(
    np.rint(np.nan_to_num(motor_current_relative, nan=0.0) * 65536),
    0,
    np.iinfo(np.uint32).max,
).astype(np.uint32)
print(to_c_array_2d_u32("motor_current_relative_lut", motor_current_relative_export))

motor_current_relative_python = np.nan_to_num(motor_current_relative, nan=0.0)
print(to_python_array_2d("motor_current_relative", motor_current_relative_python))

# 2D plot of the LUT as a heatmap
plt.figure(figsize=(8, 5))
plt.imshow(
    lut,
    origin="lower",
    aspect="auto",
    extent=[torque_axis[0], torque_axis[-1], cadence_axis[0], cadence_axis[-1]],
    cmap="viridis",
)
plt.colorbar(label="Target Motor Power (W)")
plt.xlabel("Torque (Nm)")
plt.ylabel("Cadence (RPM)")
plt.title("Assist LUT (Cadence vs Torque)")
plt.tight_layout()
plt.show()

# 2D plot of motor current
plt.figure(figsize=(8, 5))
plt.imshow(
    motor_current,
    origin="lower",
    aspect="auto",
    extent=[torque_axis[0], torque_axis[-1], cadence_axis[0], cadence_axis[-1]],
    cmap="inferno",
)
plt.colorbar(label="Motor Current (A)")
plt.xlabel("Torque (Nm)")
plt.ylabel("Cadence (RPM)")
plt.title("Motor Current")
plt.tight_layout()
plt.show()

# 2D plot of relative motor current
plt.figure(figsize=(8, 5))
plt.imshow(
    motor_current_relative,
    origin="lower",
    aspect="auto",
    extent=[torque_axis[0], torque_axis[-1], cadence_axis[0], cadence_axis[-1]],
    cmap="inferno",
)
plt.colorbar(label="Relative Motor Current (0 to 1)")
plt.xlabel("Torque (Nm)")
plt.ylabel("Cadence (RPM)")
plt.title("Relative Motor Current")
plt.tight_layout()
plt.show()

# 2D plot of motor power divided by (cadence * torque)
plt.figure(figsize=(8, 5))
plt.imshow(
    power_over_cadence_torque,
    origin="lower",
    aspect="auto",
    extent=[torque_axis[0], torque_axis[-1], cadence_axis[0], cadence_axis[-1]],
    cmap="plasma",
)
plt.colorbar(label="Motor Power / (Cadence * Torque)")
plt.xlabel("Torque (Nm)")
plt.ylabel("Cadence (RPM)")
plt.title("Normalized Assist: Power / (Cadence * Torque)")
plt.tight_layout()
plt.show()