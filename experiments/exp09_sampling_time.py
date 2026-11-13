"""Section 7d: vary the controller interval, not the integration accuracy."""

from experiments.common import base, save, target, trace
from metrics import print_metrics, response_metrics
from plant import run_response


def main():
    fig, ax = base("7d. Controller update interval (plant step ≤ 2 ms)")
    for dt in (0.01, 0.1, 0.15):
        run = run_response(100, 100, 15, duration=5, dt=dt)
        trace(ax, run, f"dt={dt:g} s")
        print_metrics(f"dt={dt:g} s", response_metrics(run))
    target(ax, 5)
    ax.legend()
    save(fig, "07d_sampling.png", "Changes arise from the controller update rate.")


if __name__ == "__main__":
    main()
