"""Section 7b: isolate derivative noise after the plant has settled."""

import numpy as np

from experiments.common import base, save
from plant import run_response


def main():
    fig, ax = base(
        "7b. ±1 mm sensor noise: steady-state derivative force",
        ylabel="Derivative force (N)",
    )
    configurations = (
        ("Unfiltered (tau=0)", 0.0),
        ("Filtered (tau=0.05 s)", 0.05),
    )
    standard_deviations = []

    for label, tau in configurations:
        run = run_response(
            100, 100, 15, duration=8, noise=0.001, filter_tau=tau,
        )
        time = run["time"]
        derivative = run["d_term"]

        # Show one second after the initial motion has settled.
        visible = (time >= 5) & (time <= 6)
        ax.plot(time[visible], derivative[visible], label=label, linewidth=1.3)

        # Use a longer window for more representative noise statistics.
        steady = derivative[time >= 5]
        deviation = np.std(steady)
        standard_deviations.append(deviation)
        print(f"{label}: D std={deviation:.4f} N, "
              f"peak-to-peak={np.ptp(steady):.4f} N (5–8 s)")

    unfiltered, filtered = standard_deviations
    reduction = 100 * (1 - filtered / unfiltered)
    print(f"Derivative noise reduction: {reduction:.2f}%")

    ax.set_xlim(5, 6)
    ax.axhline(0, color="black", linestyle="--", linewidth=1)
    ax.margins(y=0.20)
    ax.legend(loc="upper right")
    save(
        fig, "07b_noise.png",
        "Same seeded noise; 5–6 s view, statistics over 5–8 s.",
    )


if __name__ == "__main__":
    main()
