"""Section 7c: compare derivative on error and on measurement."""

import numpy as np

from experiments.common import base, save
from plant import run_response


def main():
    fig, ax = base("7c. Sudden target step at 1 s: derivative kick",
                   ylabel="Derivative force (N)")

    def setpoint(t):
        return 0.0 if t < 1.0 else 1.0

    for label, on_measurement in (("Derivative of error", False),
                                  ("Derivative of measurement", True)):
        run = run_response(100, 100, 15, duration=2, dt=0.01, target=setpoint,
                           derivative_on_measurement=on_measurement)
        ax.step(run["time"], run["d_term"], where="post", label=label, linewidth=1.7)
        index = np.searchsorted(run["time"], 1.0)
        print(f"{label}: D at step={run['d_term'][index]:.2f} N, "
              f"controller force={run['force'][index]:.2f} N")
    ax.set_xlim(0.9, 1.2)
    ax.legend()
    save(fig, "07c_kick.png", "Derivative of error creates a 1500 N one-update spike.")


if __name__ == "__main__":
    main()
