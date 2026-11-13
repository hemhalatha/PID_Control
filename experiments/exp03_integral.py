"""Section 5b: compare P and PI control."""

from experiments.common import base, save, target, trace
from metrics import print_metrics, response_metrics
from plant import run_response


def main():
    fig, ax = base("5b. Adding integral action (P vs PI)")
    for label, gains in (("P: 100, 0, 0", (100, 0, 0)),
                         ("PI: 100, 120, 0", (100, 120, 0))):
        run = run_response(*gains, duration=5)
        trace(ax, run, label)
        print_metrics(label, response_metrics(run))
    target(ax, 5)
    ax.legend()
    save(fig, "05b_integral.png", "Integral action eliminates the remaining error.")


if __name__ == "__main__":
    main()
