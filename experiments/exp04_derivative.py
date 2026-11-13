"""Section 5c: add derivative action to the PI controller."""

from experiments.common import base, save, target, trace
from metrics import print_metrics, response_metrics
from plant import run_response


def main():
    fig, ax = base("5c. Adding derivative action to PI reduces overshoot")
    for label, gains in (("PI: 100, 120, 0", (100, 120, 0)),
                         ("PID: 100, 120, 20", (100, 120, 20))):
        run = run_response(*gains, duration=5)
        trace(ax, run, label)
        print_metrics(label, response_metrics(run))
    target(ax, 5)
    ax.legend()
    save(fig, "05c_derivative.png", "Derivative action reduces the peak for these gains.")


if __name__ == "__main__":
    main()
