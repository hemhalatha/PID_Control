
from pathlib import Path

import matplotlib.pyplot as plt

from pid_controller import PIDController
from plant import MassSpringDamper, simulate
from metrics import calculate_metrics, print_metrics


def main():
    # Configure the plant and PID controller
    plant = MassSpringDamper(
        mass=1.0,
        damping=10.0,
        spring=20.0,
    )

    controller = PIDController(
        kp=100,
        ki=250,
        kd=2,
        dt=0.01,
    )

    target = 1.0
    duration = 10.0

    # Run the simulation
    results = simulate(
        controller,
        plant,
        duration=duration,
        setpoint=target,
        plant_dt=0.001,
    )

    time = results["time"]
    position = results["position"]

    # Calculate performance metrics
    metrics = calculate_metrics(time, position, target)

    print_metrics("PID Step Response", metrics)

    t10 = metrics["t10"]
    t90 = metrics["t90"]
    rise_time = metrics["rise_time"]

    peak_time = metrics["peak_time"]
    peak_position = metrics["peak_position"]

    overshoot = metrics["overshoot"]
    settling_time = metrics["settling_time"]
    steady_state_error = metrics["steady_state_error"]

    # Create the figure
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(
        time,
        position,
        color="royalblue",
        linewidth=2,
        label="PID response",
    )

    # Target position
    ax.axhline(
        target,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label="Setpoint (1 m)",
    )

    # Settling tolerance band
    ax.axhspan(
        0.98,
        1.02,
        color="green",
        alpha=0.12,
        label="±2% settling band",
    )

    # 1. Rise time
    if rise_time is not None:
        ax.scatter(
            [t10, t90],
            [0.1, 0.9],
            color="darkorange",
            zorder=5,
        )

        ax.vlines(
            [t10, t90],
            ymin=0,
            ymax=[0.1, 0.9],
            color="darkorange",
            linestyle=":",
        )

        ax.annotate(
            f"Rise time = {rise_time:.3f} s",
            xy=(t90, 0.9),
            xytext=(1.5, 0.45),
            arrowprops=dict(
                arrowstyle="->",
                color="darkorange",
            ),
            color="darkorange",
            fontsize=10,
        )

    # 2. Overshoot
    if overshoot > 0:
        ax.scatter(
            peak_time,
            peak_position,
            color="red",
            zorder=5,
        )

        ax.annotate(
            f"Overshoot = {overshoot:.2f}%",
            xy=(peak_time, peak_position),
            xytext=(1.5, peak_position + 0.12),
            arrowprops=dict(
                arrowstyle="->",
                color="red",
            ),
            color="red",
            fontsize=10,
        )

    # 3. Settling time
    if settling_time is not None:
        ax.axvline(
            settling_time,
            color="green",
            linestyle="--",
            linewidth=1.5,
            label=f"Settling time = {settling_time:.3f} s",
        )

    # 4. Steady-state error
    ax.scatter(
        time[-1],
        position[-1],
        color="purple",
        zorder=5,
    )

    ax.annotate(
        f"Steady-state error = {steady_state_error:.6f} m",
        xy=(time[-1], position[-1]),
        xytext=(5.0, 0.75),
        arrowprops=dict(
            arrowstyle="->",
            color="purple",
        ),
        color="purple",
        fontsize=10,
    )

    # Graph formatting
    ax.set_title(
        "PID Step Response - Performance Measures",
        fontsize=14,
        fontweight="bold",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")

    ax.set_xlim(0, duration + 0.2)
    ax.set_ylim(0, max(1.4, peak_position + 0.25))

    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")

    fig.tight_layout()

    # Save the graph in the project graphs directory
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "01_step_response.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
