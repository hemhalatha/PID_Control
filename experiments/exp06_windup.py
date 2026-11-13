"""Section 7a: compare saturated PID with and without anti-windup."""

import numpy as np

from experiments.common import base, save, target, trace
from metrics import print_metrics, response_metrics
from plant import run_response


def main():
    fig, ax = base("7a. Saturation ±30 N: windup versus anti-windup")
    for label, enabled in (("No anti-windup", False), ("With anti-windup", True)):
        run = run_response(70, 180, 12, duration=7, output_limit=30,
                           anti_windup=enabled)
        met = response_metrics(run)
        trace(ax, run, f"{label}; overshoot {met['overshoot']:.1f}%")
        print_metrics(label, met)
        print(f"  Peak integral force: {np.max(run['i_term']):.3f} N; "
              f"peak applied force: {np.max(np.abs(run['force'])):.3f} N")
    target(ax, 7)
    ax.legend(fontsize=9)
    save(fig, "07a_windup.png", "Both controllers are limited to ±30 N.")


if __name__ == "__main__":
    main()
