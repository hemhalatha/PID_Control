"""Section 8b: tune from sampled oscillations and refine the PID gains."""

import numpy as np

from experiments.common import base, save, target, trace
from experiments.exp10_ziegler_nichols import (
    sampled_ultimate_values,
    ziegler_nichols_gains,
)
from metrics import print_metrics, response_metrics
from plant import ThirdOrderPlant, run_response


def main():
    dt = 0.01
    ku, tu = sampled_ultimate_values(dt)
    sampled_gains = ziegler_nichols_gains(ku, tu)
    print(f"Sampled Ku={ku:.6f}, measured Tu={tu:.6f} s")
    print(f"{'Controller':<12}{'Kp':>10}{'Ki':>10}{'Kd':>10}")
    for name, (kp, ki, kd) in sampled_gains.items():
        print(f"{name:<12}{kp:>10.4f}{ki:>10.4f}{kd:>10.4f}")

    # Retain the continuous-time recipe as a reference for the original report.
    theoretical = ziegler_nichols_gains(8.0, 2 * np.pi / np.sqrt(3))["PID"]
    improved = (3.0, 1.0, 1.5)
    configurations = (
        ("Sampled ZN", sampled_gains["PID"], "-"),
        ("Theoretical ZN", theoretical, ":"),
        ("Improved", improved, "-"),
    )

    plant = ThirdOrderPlant()
    fig, ax = base(
        "8b. Ziegler–Nichols PID versus improved tuning",
        ylabel="Output (normalized units)",
    )
    performance = {}
    for label, gains, style in configurations:
        run = run_response(*gains, plant=plant, duration=18, dt=dt)
        met = response_metrics(run)
        performance[label] = met
        gain_text = ", ".join(f"{g:.2f}" for g in gains)
        trace(
            ax, run,
            f"{label} ({gain_text}); overshoot {met['overshoot']:.1f}%",
            linestyle=style,
        )
        print_metrics(label, met)

    baseline = performance["Sampled ZN"]
    refined = performance["Improved"]
    reduction = 100 * (1 - refined["overshoot"] / baseline["overshoot"])
    settling_reduction = 100 * (
        1 - refined["settling_time"] / baseline["settling_time"]
    )
    print(f"Overshoot reduction from sampled ZN: {reduction:.2f}%")
    print(f"Settling time reduction from sampled ZN: {settling_reduction:.2f}%")

    target(ax, 18, units="")
    ax.legend(fontsize=8)
    save(
        fig, "08b_zn_tuning.png",
        "Measured ZN and theoretical reference compared with manual retuning.",
    )


if __name__ == "__main__":
    main()
