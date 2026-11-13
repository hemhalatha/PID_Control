
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pid_controller import PIDController
from plant import MassSpringDamper, simulate
from metrics import calculate_metrics


def main():
    # Plant and simulation parameters
    mass = 1.0
    damping = 10.0
    spring = 20.0

    target = 1.0
    duration = 10.0
    dt = 0.01

    kp_values = [10, 50, 100, 300]

    plant = MassSpringDamper(mass, damping, spring)

    fig, ax = plt.subplots(figsize=(11, 6))

    # Print comparison table header
    print("\nProportional Control - Steady-State Analysis")
    print("-" * 88)

    print(
        f"{'Kp':>6}"
        f"{'Theory (m)':>15}"
        f"{'Simulated (m)':>17}"
        f"{'Error (m)':>14}"
        f"{'Overshoot (%)':>16}"
        f"{'Verified':>12}"
    )

    print("-" * 88)

    for kp in kp_values:
        # Configure proportional-only controller
        controller = PIDController(
            kp=kp,
            ki=0.0,
            kd=0.0,
            dt=dt,
        )

        # Run simulation
        results = simulate(
            controller,
            plant,
            duration=duration,
            setpoint=target,
            plant_dt=0.001,
        )

        time = results["time"]
        position = results["position"]

        # Calculate theoretical steady-state values
        theoretical_position = (
            kp / (kp + spring)
        ) * target

        theoretical_error = target - theoretical_position

        # Calculate simulated performance metrics
        metrics = calculate_metrics(time, position, target)

        simulated_position = metrics["final_position"]
        simulated_error = metrics["steady_state_error"]
        overshoot = metrics["overshoot"]

        # Verify theoretical and simulated results
        verified = np.isclose(
            simulated_position,
            theoretical_position,
            atol=1e-5,
            rtol=0.0,
        )

        status = "Yes" if verified else "No"

        print(
            f"{kp:>6}"
            f"{theoretical_position:>15.6f}"
            f"{simulated_position:>17.6f}"
            f"{simulated_error:>14.6f}"
            f"{overshoot:>16.2f}"
            f"{status:>12}"
        )

        # Plot position response
        line, = ax.plot(
            time,
            position,
            linewidth=2,
            label=f"Kp = {kp}",
        )

        # Mark the theoretical equilibrium position
        ax.axhline(
            theoretical_position,
            color=line.get_color(),
            linestyle=":",
            linewidth=1,
            alpha=0.6,
        )

        # Label the theoretical steady-state error
        ax.text(
            duration + 0.1,
            theoretical_position,
            f"e = {theoretical_error:.3f} m",
            color=line.get_color(),
            fontsize=9,
            verticalalignment="center",
        )

    print("-" * 88)

    # Target position
    ax.axhline(
        target,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label="Setpoint (1 m)",
    )

    # Graph formatting
    ax.set_title(
        "Effect of Proportional Gain on Step Response",
        fontsize=14,
        fontweight="bold",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")

    ax.set_xlim(0, duration + 1.4)
    ax.set_ylim(0, 1.5)

    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "02_proportional.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()