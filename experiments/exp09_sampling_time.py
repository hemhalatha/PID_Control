
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
    plant_dt = 0.001

    kp = 50
    ki = 100
    kd = 5

    sampling_times = [0.01, 0.05, 0.1, 0.2, 0.5]

    fig, axes = plt.subplots(
        2, 2, figsize=(14, 10)
    )

    ax1, ax2 = axes[0]
    ax3, ax4 = axes[1]

    print("\nEffect of Controller Sampling Time")
    print("-" * 100)

    print(
        f"{'dt (s)':>8}"
        f"{'Frequency (Hz)':>17}"
        f"{'Rise (s)':>12}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Final Error (m)':>18}"
    )

    print("-" * 100)

    for dt in sampling_times:
        # Configure the controller with a different sampling time
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            derivative_on_measurement=True,
        )

        # Keep the plant integration step unchanged
        results = simulate(
            controller,
            plant,
            duration=duration,
            setpoint=target,
            plant_dt=plant_dt,
        )

        time = results["time"]
        position = results["position"]
        force = results["force"]

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

        frequency = 1.0 / dt

        print(
            f"{dt:>8.3f}"
            f"{frequency:>17.1f}"
            f"{rise_text:>12}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['steady_state_error']:>18.6f}"
        )

        label = f"dt = {dt:g} s ({frequency:g} Hz)"

        # Compare the stable responses separately
        # so the unstable response does not hide them.
        if dt < 0.5:
            line, = ax1.plot(
                time,
                position,
                linewidth=2,
                label=label,
            )

            ax2.step(
                time,
                force,
                where="post",
                color=line.get_color(),
                linewidth=1.5,
                label=label,
            )

        # Compare all responses during the first two seconds
        ax3.plot(
            time,
            position,
            linewidth=1.8,
            label=label,
        )

        # Plot absolute error on a logarithmic scale
        # to reveal both convergence and divergence.
        absolute_error = np.abs(target - position)

        absolute_error = np.maximum(
            absolute_error, 1e-12
        )

        ax4.semilogy(
            time,
            absolute_error,
            linewidth=1.8,
            label=label,
        )

    print("-" * 100)

    # Plot 1: Stable position responses
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

    ax1.set_title("Position Response: Faster Updates")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.set_xlim(0, 4)
    ax1.legend(fontsize=8)

    # Plot 2: Applied force with different update intervals
    ax2.axhline(
        20,
        color="black",
        linestyle="--",
        linewidth=1,
        label="Holding force (20 N)",
    )

    ax2.set_title("Control Force and Zero-Order Hold")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.set_xlim(0, 2)
    ax2.legend(fontsize=8)

    # Plot 3: Include the unstable response
    ax3.axhline(
        target,
        color="black",
        linestyle="--",
        label="Setpoint (1 m)",
    )

    ax3.set_title("Effect of Very Slow Sampling")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position (m)")
    ax3.set_xlim(0, 2)
    ax3.legend(fontsize=8)

    # Plot 4: Error convergence and divergence
    ax4.axhline(
        0.02,
        color="black",
        linestyle="--",
        linewidth=1,
        label="2% error threshold",
    )

    ax4.set_title("Absolute Error Over Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Absolute error (m)")
    ax4.set_xlim(0, duration)
    ax4.legend(fontsize=8)

    # Common formatting
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "Effect of Controller Sampling Time on PID Performance",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "09_sampling_time.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
