
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pid_controller import PIDController
from plant import MassSpringDamper, simulate
from metrics import calculate_metrics


def main():
    # Plant and simulation parameters
    plant = MassSpringDamper(
        mass=1.0,
        damping=10.0,
        spring=20.0,
    )

    target = 1.0
    duration = 10.0
    dt = 0.01
    force_limit = 30.0

    kp = 50
    ki = 100
    kd = 5

    configurations = {
        "Without anti-windup": False,
        "With anti-windup": True,
    }

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(11, 10), sharex=True
    )

    print("\nIntegral Windup Comparison")
    print("-" * 85)

    print(
        f"{'Controller':<24}"
        f"{'Rise (s)':>12}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Peak I (N)':>14}"
    )

    print("-" * 85)

    for name, anti_windup in configurations.items():
        # Configure the PID controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            output_limits=(-force_limit, force_limit),
            anti_windup=anti_windup,
            derivative_on_measurement=True,
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
        force = results["force"]

        p_term = results["p_term"]
        i_term = results["i_term"]
        d_term = results["d_term"]

        # Controller output before actuator saturation
        requested_force = p_term + i_term + d_term

        # Calculate performance metrics
        metrics = calculate_metrics(
            time, position, target
        )

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

        peak_integral = np.max(i_term)

        print(
            f"{name:<24}"
            f"{rise_text:>12}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{peak_integral:>14.2f}"
        )

        # Position response
        line, = ax1.plot(
            time,
            position,
            linewidth=2,
            label=name,
        )

        color = line.get_color()

        # Actual force applied to the plant
        ax2.plot(
            time,
            force,
            color=color,
            linewidth=2,
            label=f"{name}: actual",
        )

        # Requested force before saturation
        ax2.plot(
            time,
            requested_force,
            color=color,
            linestyle=":",
            linewidth=1.5,
            label=f"{name}: requested",
        )

        # Integral contribution
        ax3.plot(
            time,
            i_term,
            color=color,
            linewidth=2,
            label=name,
        )

    print("-" * 85)

    # Position response
    ax1.axhline(
        target,
        color="black",
        linestyle="--",
        label="Setpoint (1 m)",
    )

    ax1.axhspan(
        0.98,
        1.02,
        color="green",
        alpha=0.1,
        label="±2% settling band",
    )

    ax1.set_title("Position Response")
    ax1.set_ylabel("Position (m)")
    ax1.legend(fontsize=9)

    # Actuator force
    ax2.axhline(
        force_limit,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label="Force limit (+30 N)",
    )

    ax2.axhline(
        -force_limit,
        color="red",
        linestyle="--",
        linewidth=1.2,
    )

    ax2.set_title("Requested and Applied Force")
    ax2.set_ylabel("Force (N)")
    ax2.legend(fontsize=9, ncol=2)

    # Integral contribution
    ax3.axhline(
        20,
        color="black",
        linestyle="--",
        linewidth=1.2,
        label="Required holding force (20 N)",
    )

    ax3.set_title("Integral Contribution")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Integral force (N)")
    ax3.legend(fontsize=9)

    # Common graph formatting
    for ax in (ax1, ax2, ax3):
        ax.set_xlim(0, 5)
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "Integral Windup and Anti-Windup Control",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "06_windup.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
