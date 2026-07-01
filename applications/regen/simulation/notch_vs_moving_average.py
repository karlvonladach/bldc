import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np


# LMS notch filter state (equivalent to the C++ example)
A = 0.0
B = 0.0


def filter_torque_lms(torque_raw: float, theta_rad: float, mu: float) -> float:
    """Adaptive notch filter step, matching the attached C++ logic."""
    global A, B

    x1 = math.sin(theta_rad)
    x2 = math.cos(theta_rad)

    y_estimated = (A * x1) + (B * x2)
    torque_filtered = torque_raw - y_estimated

    A = A + (mu * torque_filtered * x1)
    B = B + (mu * torque_filtered * x2)

    return torque_filtered


def moving_average_causal(signal: np.ndarray, window_size: int) -> np.ndarray:
    """Causal moving average with fixed window size."""
    out = np.zeros_like(signal)
    window = deque()
    running_sum = 0.0

    for i, value in enumerate(signal):
        window.append(value)
        running_sum += value

        if len(window) > window_size:
            running_sum -= window.popleft()

        out[i] = running_sum / len(window)

    return out


def main() -> None:
    # Simulation settings
    samples_per_period = 72
    total_periods = 8
    zero_periods = 2
    tail_zero_periods = 2
    mu = 0.1

    total_samples = samples_per_period * (zero_periods + total_periods + tail_zero_periods)
    theta = 2.0 * math.pi * np.arange(total_samples) / samples_per_period

    # Test signal: starts with 0, then oscillatory segment, then returns to 0
    raw_signal = np.zeros(total_samples)
    start_index = zero_periods * samples_per_period
    end_index = (zero_periods + total_periods) * samples_per_period
    raw_signal[start_index:end_index] = (
        0.5
        - 0.5 * np.cos(theta[start_index:end_index])
        + 0.01 * np.sin(0.5 * theta[start_index:end_index])
    )

    # Run LMS notch filter sample-by-sample
    global A, B
    A = 0.0
    B = 0.0
    notch_output = np.zeros_like(raw_signal)
    for i in range(total_samples):
        notch_output[i] = filter_torque_lms(float(raw_signal[i]), float(theta[i]), mu)

    # Moving average with the same period length
    ma_output = moving_average_causal(raw_signal, window_size=samples_per_period)

    # Plot all signals
    time_idx = np.arange(total_samples)
    plt.figure(figsize=(12, 7))

    plt.plot(time_idx, raw_signal, label="Raw test signal", linewidth=2.0, alpha=0.8)
    plt.plot(time_idx, notch_output, label=f"Notch output (mu={mu})", linewidth=2.0)
    plt.plot(
        time_idx,
        ma_output,
        label=f"Moving average output (window={samples_per_period})",
        linewidth=2.0,
    )

    plt.axvline(start_index, color="black", linestyle="--", alpha=0.6, label="Signal starts")
    plt.axvline(end_index, color="gray", linestyle="--", alpha=0.6, label="Signal ends")
    plt.title("Notch vs Moving Average on Step-to-Cosine Test Signal")
    plt.xlabel("Sample index")
    plt.ylabel("Torque")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
