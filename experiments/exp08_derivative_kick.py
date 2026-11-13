
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pid_controller import PIDController
from plant import MassSpringDamper, simulate


def main():
    # Plant and simulation parameters
    plant = MassSpringDamper(
        mass=1.0,
        damping=10.0,
        spring=20.0,
    )

    duration = 5.0
    dt = 0.01
    step_time = 1.0
    target = 1.0

    kp = 50
    ki = 100
    kd = 5

    # Setpoint changes from 0 m to 1 m at t = 1 s.
    def setpoint(t):
        return target if t >= step_time else 0.0

    configurations = {
        "Derivative on error": False,
        "Derivative on measurement": True,
    }

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(11, 10)
    )

    print("\nDerivative Kick Comparison")
    print("-" * 86)

    print(
        f"{'Controller':<30}"
        f"{'D at Step (N)':>18}"
        f"{'Peak |D| (N)':>18}"
        f"{'Force at Step (N)':>20}"
    )

    print("-" * 86)

    for name, derivative_on_measurement in configurations.items():

        # Configure the PID controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            derivative_on_measurement=derivative_on_measurement,
        )

        # Run the simulation
        results = simulate(
            controller,
            plant,
            duration=duration,
            setpoint=setpoint,
            plant_dt=0.001,
        )

        time = results["time"]
        position = results["position"]
        derivative = results["d_term"]
        force = results["force"]

        # Find the sample where the setpoint changes
        step_index = np.searchsorted(time, step_time)

        derivative_at_step = derivative[step_index]
        force_at_step = force[step_index]

        peak_derivative = np.max(
            np.abs(derivative)
        )

        print(
            f"{name:<30}"
            f"{derivative_at_step:>18.2f}"
            f"{peak_derivative:>18.2f}"
            f"{force_at_step:>20.2f}"
        )

        # Position response
        line, = ax1.plot(
            time,
            position,
            linewidth=2,
            label=name,
        )

        color = line.get_color()

        # Derivative contribution
        ax2.step(
            time,
            derivative,
            where="post",
            color=color,
            linewidth=1.5,
            label=name,
        )

        # Total controller force
        ax3.step(
            time,
            force,
            where="post",
            color=color,
            linewidth=1.5,
            label=name,
        )

        # Mark the derivative value at the setpoint change
        ax2.scatter(
            step_time,
            derivative_at_step,
            color=color,
            zorder=5,
        )

    print("-" * 86)

    # Position response
    ax1.step(
        time,
        results["setpoint"],
        where="post",
        color="black",
        linestyle="--",
        linewidth=1.5,
        label="Setpoint",
    )

    ax1.set_title("Position Response to a Setpoint Change")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.set_xlim(0, duration)
    ax1.legend(fontsize=9)

    # Derivative contribution
    ax2.axhline(
        0,
        color="black",
        linestyle="--",
        linewidth=1,
    )

    ax2.set_title("Derivative Kick")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Derivative force (N)")
    ax2.set_xlim(0.9, 1.5)

    # Label the theoretical derivative spike
    ax2.annotate(
        "500 N derivative kick",
        xy=(step_time, 500),
        xytext=(1.12, 400),
        arrowprops=dict(
            arrowstyle="->",
            color="black",
        ),
        fontsize=10,
    )

    ax2.legend(fontsize=9)

    # Total controller force
    ax3.axhline(
        30,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label="30 N actuator limit (reference)",
    )

    ax3.set_title("Controller Output Force")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Force (N)")
    ax3.set_xlim(0.9, 1.5)

    ax3.annotate(
        "551 N initial force",
        xy=(step_time, 551),
        xytext=(1.12, 450),
        arrowprops=dict(
            arrowstyle="->",
            color="black",
        ),
        fontsize=10,
    )

    ax3.legend(fontsize=9)

    # Mark the setpoint change on all plots
    for ax in (ax1, ax2, ax3):
        ax.axvline(
            step_time,
            color="gray",
            linestyle=":",
            linewidth=1,
        )

        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "Derivative Kick and Derivative on Measurement",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "08_derivative_kick.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
