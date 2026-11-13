
from pathlib import Path

import matplotlib.pyplot as plt

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

    kp = 50
    ki = 300
    kd_values = [0, 2, 5, 10, 20]

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(14, 6)
    )

    print("\nEffect of Derivative Gain")
    print("-" * 78)

    print(
        f"{'Kd':>6}"
        f"{'Rise (s)':>14}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Final Error (m)':>18}"
    )

    print("-" * 78)

    for kd in kd_values:
        # Configure PI or PID controller
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
            plant_dt=0.001,
        )

        time = results["time"]
        position = results["position"]
        velocity = results["velocity"]

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

        print(
            f"{kd:>6}"
            f"{rise_text:>14}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['steady_state_error']:>18.6f}"
        )

        # Generate controller label
        if kd == 0:
            label = "PI only"
        else:
            label = f"PID: Kd = {kd}"

        # Plot position response
        line, = ax1.plot(
            time,
            position,
            linewidth=2,
            label=label,
        )

        # Plot velocity response
        ax2.plot(
            time,
            velocity,
            color=line.get_color(),
            linewidth=2,
            label=label,
        )

    print("-" * 78)

    # Position plot
    ax1.axhline(
        target,
        color="black",
        linestyle="--",
        linewidth=1.5,
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
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.set_xlim(0, 5)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)

    # Velocity plot
    ax2.axhline(
        0,
        color="black",
        linestyle="--",
        linewidth=1,
        label="Zero velocity",
    )

    ax2.set_title("Velocity Response")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_xlim(0, 5)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)

    fig.suptitle(
        "Effect of Derivative Gain (Kp = 50, Ki = 300)",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "04_derivative.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
