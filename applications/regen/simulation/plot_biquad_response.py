import argparse
import numpy as np
import matplotlib.pyplot as plt


def normalize_coefficients(b, a):
    """Normalize coefficients so that a0 == 1.0."""
    if a[0] == 0.0:
        raise ValueError("a0 must not be zero")
    if a[0] != 1.0:
        b = b / a[0]
        a = a / a[0]
    return b, a


def biquad_frequency_response(b, a, fs_hz, points=4096):
    """Compute H(e^jw) for a biquad filter."""
    # Frequencies from 0 to Nyquist
    f = np.linspace(0.1, fs_hz / 2.0, points)
    w = 2.0 * np.pi * f / fs_hz
    z_inv = np.exp(-1j * w)

    num = b[0] + b[1] * z_inv + b[2] * (z_inv ** 2)
    den = a[0] + a[1] * z_inv + a[2] * (z_inv ** 2)
    h = num / den

    magnitude_db = 20.0 * np.log10(np.maximum(np.abs(h), 1e-15))
    phase_deg = np.unwrap(np.angle(h)) * 180.0 / np.pi

    return f, magnitude_db, phase_deg


def phase_deg_to_time_delay_ms(phase_deg, frequency_hz):
    """Convert phase offset to equivalent time delay at each frequency."""
    # dt = -phi / omega = -(phase_deg / 360) / f
    delay_s = -(phase_deg / 360.0) / frequency_hz
    return delay_s * 1000.0


def bilinear_second_order(b_analog, a_analog, fs_hz):
    """Bilinear transform for a 2nd-order analog transfer function."""
    k = 2.0 * fs_hz

    b0, b1, b2 = b_analog
    a0, a1, a2 = a_analog

    b_dig = np.array(
        [
            b0 * k * k + b1 * k + b2,
            2.0 * (b2 - b0 * k * k),
            b0 * k * k - b1 * k + b2,
        ],
        dtype=float,
    )
    a_dig = np.array(
        [
            a0 * k * k + a1 * k + a2,
            2.0 * (a2 - a0 * k * k),
            a0 * k * k - a1 * k + a2,
        ],
        dtype=float,
    )

    return normalize_coefficients(b_dig, a_dig)


def design_two_real_pole_lpf(fs_hz, pole1_hz, pole2_hz):
    """Design LPF with analog poles at -2*pi*pole1_hz and -2*pi*pole2_hz."""
    w1 = 2.0 * np.pi * pole1_hz
    w2 = 2.0 * np.pi * pole2_hz

    # H(s) = (w1*w2) / ((s + w1)(s + w2)) gives unity DC gain.
    b_analog = np.array([0.0, 0.0, w1 * w2], dtype=float)
    a_analog = np.array([1.0, w1 + w2, w1 * w2], dtype=float)
    return bilinear_second_order(b_analog, a_analog, fs_hz)


def main():
    parser = argparse.ArgumentParser(
        description="Plot magnitude and phase response of a biquad filter."
    )
    parser.add_argument(
        "--fs",
        type=float,
        default=500.0,
        help="Sampling frequency in Hz (default: 500).",
    )
    parser.add_argument(
        "--coeff",
        type=float,
        nargs=5,
        metavar=("b0", "b1", "b2", "a1", "a2"),
        default=[
            0.000609854721,
            0.001219709442,
            0.000609854721,
            -1.928942259604,
            0.931381678488,
        ],
        help=(
            "Biquad coefficients in the form: b0 b1 b2 a1 a2. "
            "a0 is assumed to be 1.0."
        ),
    )
    args = parser.parse_args()

    b0, b1, b2, a1, a2 = args.coeff

    b = np.array([b0, b1, b2], dtype=float)
    a = np.array([1.0, a1, a2], dtype=float)
    b, a = normalize_coefficients(b, a)

    b_new, a_new = design_two_real_pole_lpf(args.fs, pole1_hz=1.0, pole2_hz=4.0)
    b_2hz2, a_2hz2 = design_two_real_pole_lpf(args.fs, pole1_hz=2.0, pole2_hz=2.0)

    print("Original coefficients (a0 assumed 1):")
    print(f"b0={b[0]:.12f}, b1={b[1]:.12f}, b2={b[2]:.12f}, a1={a[1]:.12f}, a2={a[2]:.12f}")
    print("New coefficients for poles at 1 Hz and 4 Hz (a0=1):")
    print(
        f"b0={b_new[0]:.12f}, b1={b_new[1]:.12f}, b2={b_new[2]:.12f}, "
        f"a1={a_new[1]:.12f}, a2={a_new[2]:.12f}"
    )
    print("New coefficients for double poles at 2 Hz (a0=1):")
    print(
        f"b0={b_2hz2[0]:.12f}, b1={b_2hz2[1]:.12f}, b2={b_2hz2[2]:.12f}, "
        f"a1={a_2hz2[1]:.12f}, a2={a_2hz2[2]:.12f}"
    )

    f, magnitude_db, phase_deg = biquad_frequency_response(b, a, args.fs)
    _, magnitude_db_new, phase_deg_new = biquad_frequency_response(b_new, a_new, args.fs)
    _, magnitude_db_2hz2, phase_deg_2hz2 = biquad_frequency_response(b_2hz2, a_2hz2, args.fs)

    delay_ms = phase_deg_to_time_delay_ms(phase_deg, f)
    delay_ms_new = phase_deg_to_time_delay_ms(phase_deg_new, f)
    delay_ms_2hz2 = phase_deg_to_time_delay_ms(phase_deg_2hz2, f)

    fig, (ax_mag, ax_delay) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax_mag.plot(f, magnitude_db, linewidth=1.5, label="Original")
    ax_mag.plot(f, magnitude_db_new, linewidth=1.5, label="New (poles at 1 Hz and 4 Hz)")
    ax_mag.plot(f, magnitude_db_2hz2, linewidth=1.5, label="New (double poles at 2 Hz)")
    ax_mag.set_title("Biquad Frequency Response Comparison")
    ax_mag.set_ylabel("Magnitude [dB]")
    ax_mag.set_xscale("log")
    ax_mag.grid(True, which="both", linestyle="--", alpha=0.6)
    ax_mag.legend()

    ax_delay.plot(f, delay_ms, linewidth=1.5, label="Original")
    ax_delay.plot(f, delay_ms_new, linewidth=1.5, label="New (poles at 1 Hz and 4 Hz)")
    ax_delay.plot(f, delay_ms_2hz2, linewidth=1.5, label="New (double poles at 2 Hz)")
    ax_delay.set_xlabel("Frequency [Hz]")
    ax_delay.set_ylabel("Time Delay [ms]")
    ax_delay.set_xscale("log")
    ax_delay.grid(True, which="both", linestyle="--", alpha=0.6)
    ax_delay.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
