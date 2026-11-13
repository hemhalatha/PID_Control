"""Section 5a: compare four proportional gains with theory."""

import numpy as np

from experiments.common import base, save, target, trace
from plant import MassSpringDamper, run_response


def main():
    plant = MassSpringDamper()
    fig, ax = base("5a. P-only control at four gains")
    for kp in (20, 50, 100, 200):
        run = run_response(kp, duration=5, plant=plant)
        final = run["position"][-1]
        theory = kp / (kp + plant.spring)
        trace(ax, run, f"Kp={kp}; final={final:.3f} m")
        verified = np.isclose(final, theory, atol=1e-5, rtol=0)
        print(f"Kp={kp}: theory={theory:.6f} m, simulation={final:.6f} m, "
              f"theoretical error={1-theory:.6f} m, verified={verified}")
    target(ax, 5)
    ax.legend(fontsize=8)
    save(fig, "05a_p_only.png", "P alone has nonzero permanent error.")


if __name__ == "__main__":
    main()
