"""Section 4: annotate the four step response measures."""

import numpy as np

from experiments.common import base, save, target, trace
from metrics import print_metrics, response_metrics
from plant import run_response


def main():
    run = run_response(100, 120, 20, duration=6)
    met = response_metrics(run)
    fig, ax = base("4. Four step-response measures (PID: 100, 120, 20)")
    trace(ax, run, "Position")
    target(ax, 6)
    ax.plot(met["peak_time"], met["peak_position"], "o",
            label=f"Peak / overshoot = {met['overshoot']:.2f}%")
    ax.axhspan(0.98, 1.02, alpha=0.10, label="±2% settling band")
    if np.isfinite(met["settling_time"]):
        ax.axvline(met["settling_time"], linestyle=":",
                   label=f"Settling time = {met['settling_time']:.2f} s")
    if np.isfinite(met["rise_time"]):
        ax.plot([met["t10"], met["t90"]], [0.1, 0.9], "s",
                label=f"10–90% rise = {met['rise_time']:.2f} s")
    error = met["steady_state_error"]
    error_text = f"{error:.3f}" if abs(error) >= 0.0005 else "0.000"
    ax.annotate(f"Final error ≈ {error_text} m",
                xy=(run["time"][-1], run["position"][-1]),
                xytext=(3.7, 0.65), arrowprops=dict(arrowstyle="->"))
    ax.legend(fontsize=8, loc="lower right")
    print_metrics("PID step response", met)
    save(fig, "04_metrics.png", "Rise, overshoot, settling and final error annotated.")


if __name__ == "__main__":
    main()
