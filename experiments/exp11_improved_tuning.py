
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pid_controller import PIDController
from plant import simulate
from metrics import calculate_metrics

from experiments.exp10_ziegler_nichols import (
    ThirdOrderPlant,
    find_ultimate_gain,
    measure_period,
    ziegler_nichols_gains,
)


def main():
    # Plant and simulation parameters
    plant = ThirdOrderPlant()

    target = 1.0
    duration = 30.0
    dt = 0.01

    # Calculate the original Ziegler-Nichols gains
    ku = find_ultimate_gain(dt)

    ultimate_controller = PIDController(
        kp=ku,
        dt=dt,
    )

    ultimate_results = simulate(
        ultimate_controller,
        plant,
        duration=50.0,
        setpoint=target,
        plant_dt=dt,
    )

    tu, _ = measure_period(
        ultimate_results["time"],
        ultimate_results["position"],
    )

    zn_gains = ziegler_nichols_gains(ku, tu)["PID"]

    # Controller configurations: (Kp, Ki, Kd)
    configurations = {
        "Original ZN PID": zn_gains,
        "Tuning attempt 1": (3.0, 1.5, 2.0),
        "Tuning attempt 2": (3.0, 1.0, 2.0),
        "Improved PID": (3.0, 0.7, 2.0),
    }

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(12, 9)
    )

    performance = {}

    print("\nImproving Ziegler-Nichols PID Tuning")
    print("-" * 112)

    print(
        f"{'Controller':<20}"
        f"{'Kp':>8}"
        f"{'Ki':>8}"
        f"{'Kd':>8}"
        f"{'Rise (s)':>12}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Final Error':>15}"
    )

    print("-" * 112)

    for name, (kp, ki, kd) in configurations.items():

        # Configure the PID controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            derivative_on_measurement=True,
        )

        # Run the simulation
        results = simulate(
            controller,
            plant,
            duration=duration,
            setpoint=target,
            plant_dt=dt,
        )

        time = results["time"]
        position = results["position"]
        control = results["force"]

        # Calculate performance metrics
        metrics = calculate_metrics(
            time, position, target
        )

        performance[name] = metrics

        rise_time = metrics["rise_time"]
        settling_time = metrics["settling_time"]

        rise_text = (
            f"{rise_time:.3f}"
            if rise_time is not None
            else "N/A"
        )

        settling_text = (
            f"{settling_time:.3f}"
            if settling_time is not None
            else "N/A"
        )

        print(
            f"{name:<20}"
            f"{kp:>8.3f}"
            f"{ki:>8.3f}"
            f"{kd:>8.3f}"
            f"{rise_text:>12}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['steady_state_error']:>15.6f}"
        )

        label = (
            f"{name} "
            f"({kp:.2f}, {ki:.2f}, {kd:.2f})"
        )

        # Plot position response
        line, = ax1.plot(
            time,
            position,
            linewidth=2,
            label=label,
        )

        # Plot control input
        ax2.plot(
            time,
            control,
            color=line.get_color(),
            linewidth=1.8,
            label=name,
        )

    print("-" * 112)

    # Compare original and improved PID performance
    original = performance["Original ZN PID"]
    improved = performance["Improved PID"]

    overshoot_reduction = (
        (original["overshoot"] - improved["overshoot"])
        / original["overshoot"]
    ) * 100

    settling_reduction = (
        (
            original["settling_time"]
            - improved["settling_time"]
        )
        / original["settling_time"]
    ) * 100

    print("\nImprovement Summary")

    print(
        f"Overshoot reduction    : "
        f"{overshoot_reduction:.2f}%"
    )

    print(
        f"Settling time reduction: "
        f"{settling_reduction:.2f}%"
    )

    print(
        f"Final absolute error   : "
        f"{abs(improved['steady_state_error']):.6f}"
    )

    # Position response plot
    ax1.axhline(
        target,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label="Setpoint (1 unit)",
    )

    ax1.axhspan(
        0.98,
        1.02,
        color="green",
        alpha=0.1,
        label="±2% settling band",
    )

    # Mark original and improved peak positions
    ax1.scatter(
        original["peak_time"],
        original["peak_position"],
        color="royalblue",
        zorder=5,
    )

    ax1.scatter(
        improved["peak_time"],
        improved["peak_position"],
        color="red",
        zorder=5,
    )

    ax1.annotate(
        f"ZN overshoot: {original['overshoot']:.2f}%",
        xy=(
            original["peak_time"],
            original["peak_position"],
        ),
        xytext=(4.5, 1.55),
        arrowprops=dict(arrowstyle="->"),
        fontsize=10,
    )

    ax1.annotate(
        f"Improved: {improved['overshoot']:.2f}%",
        xy=(
            improved["peak_time"],
            improved["peak_position"],
        ),
        xytext=(6.0, 1.15),
        arrowprops=dict(arrowstyle="->"),
        fontsize=10,
    )

    ax1.set_title("Original and Improved PID Step Responses")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Output (normalized units)")

    ax1.set_xlim(0, 15)
    ax1.set_ylim(0, 1.7)

    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Control input plot
    ax2.axhline(
        1.0,
        color="black",
        linestyle="--",
        linewidth=1.2,
        label="Equilibrium input",
    )

    ax2.set_title("Control Input Comparison")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Control input (normalized units)")

    ax2.set_xlim(0, 15)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    fig.suptitle(
        "Improving Ziegler-Nichols PID Tuning",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "11_improved_tuning.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
