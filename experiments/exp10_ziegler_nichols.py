"""Section 8a: ultimate gain for the third-order plant."""

import numpy as np

from experiments.common import base, save, target, trace
from plant import ThirdOrderPlant, rk4, run_response


def find_ultimate_gain(dt=0.01, plant_dt=0.002):
    """Locate the sampled P-loop boundary using its state-transition matrix."""
    steps = max(1, int(np.ceil(dt / plant_dt)))
    h = dt / steps

    def advance(state, u):
        for _ in range(steps):
            state = rk4(ThirdOrderPlant.derivatives, state, u, h)
        return state

    a = np.column_stack([advance(v, 0.0) for v in np.eye(3)])
    b = advance(np.zeros(3), 1.0)
    c = np.array([0.0, 0.0, 1.0])

    def radius(kp):
        return np.max(np.abs(np.linalg.eigvals(a - kp * np.outer(b, c))))

    lo, hi = 0.0, 12.0
    if radius(hi) < 1:
        raise ValueError("Ultimate gain not bracketed.")
    for _ in range(50):
        mid = (lo + hi) / 2
        if radius(mid) < 1:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def measure_period(run):
    """Average the last five peak-to-peak intervals after the initial transient."""
    t, y = run["time"], run["position"]
    peaks = np.flatnonzero((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:])) + 1
    peaks = peaks[t[peaks] >= 10]
    if len(peaks) < 2:
        raise ValueError("Not enough peaks to measure a period.")
    return float(np.mean(np.diff(t[peaks[-6:]])))


def sampled_ultimate_values(dt=0.01, plant_dt=0.002):
    """Find the sampled stability boundary and measure its oscillation period."""
    ku = find_ultimate_gain(dt, plant_dt)
    run = run_response(
        ku, dt=dt, plant_dt=plant_dt, plant=ThirdOrderPlant(), duration=50,
    )
    tu = measure_period(run)
    return ku, tu


def ziegler_nichols_gains(ku, tu):
    """Classical ultimate-gain rules converted to parallel-form gains."""
    return {
        "P": (0.5 * ku, 0.0, 0.0),
        "PI": (0.45 * ku, 0.45 * ku / (tu / 1.2), 0.0),
        "PID": (0.6 * ku, 0.6 * ku / (tu / 2), 0.6 * ku * tu / 8),
    }


def main():
    plant = ThirdOrderPlant()
    fig, ax = base("8a. Ziegler–Nichols: oscillation test for 1/(s+1)³",
                   ylabel="Output (normalized units)")
    for kp in (5, 8, 10):
        run = run_response(kp, plant=plant, duration=16)
        trace(ax, run, f"P-only Kp={kp}")

    # Keep the three original traces and add the true digital marginal case.
    ku, tu = sampled_ultimate_values()
    trace(ax, run_response(ku, plant=plant, duration=16),
          f"Sampled Ku={ku:.3f}, Tu={tu:.3f} s", linestyle=":")
    target(ax, 16, units="")
    ax.legend(fontsize=8)
    print(f"Continuous-time Ku=8, Tu={2*np.pi/np.sqrt(3):.6f} s")
    print(f"Sampled Ku={ku:.6f}, measured Tu={tu:.6f} s (dt=0.01 s)")
    print("Kp=8 is slightly above the sampled stability boundary.")
    save(fig, "08a_ultimate_gain.png", "Compare decaying, sustained and growing oscillations.")


if __name__ == "__main__":
    main()
