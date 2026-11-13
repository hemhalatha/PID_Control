
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
    duration = 30.0
    dt = 0.01

    kp = 50
    ki_values = [0, 20, 100, 300, 600]

    # Create full response and transient response plots
    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(14, 6)
    )

    print("\nEffect of Integral Gain")
    print("-" * 86)

    print(
        f"{'Ki':>6}"
        f"{'Rise (s)':>14}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Final Pos (m)':>17}"
        f"{'Error (m)':>14}"
    )

    print("-" * 86)

    for ki in ki_values:
        # Configure P or PI controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=0.0,
            dt=dt,
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
            f"{ki:>6}"
            f"{rise_text:>14}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['final_position']:>17.6f}"
            f"{metrics['steady_state_error']:>14.6f}"
        )

        # Generate controller label
        if ki == 0:
            label = "P only"
        else:
            label = f"PI: Ki = {ki}"

        # Plot the complete response
        line, = ax1.plot(
            time,
            position,
            linewidth=2,
            label=label,
        )

        # Plot the same response over a shorter time range
        ax2.plot(
            time,
            position,
            color=line.get_color(),
            linewidth=2,
            label=label,
        )

    print("-" * 86)

    # Format both plots
    for ax in (ax1, ax2):
        ax.axhline(
            target,
            color="black",
            linestyle="--",
            linewidth=1.5,
            label="Setpoint (1 m)",
        )

        ax.axhspan(
            0.98,
            1.02,
            color="green",
            alpha=0.1,
            label="±2% settling band",
        )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")

        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

    ax1.set_title("Complete Step Response")
    ax1.set_xlim(0, duration)

    ax2.set_title("Transient Response (First 5 Seconds)")
    ax2.set_xlim(0, 5)

    fig.suptitle(
        "Effect of Integral Gain on Step Response (Kp = 50)",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "03_integral.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
