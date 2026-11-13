
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
    duration = 15.0
    dt = 0.01

    # Controller configurations: (Kp, Ki, Kd)
    configurations = {
        "P": (50, 0, 0),
        "PI": (50, 100, 0),
        "PD": (50, 0, 5),
        "PID": (50, 100, 5),
        "Tuned PID": (120, 250, 5),
    }

    fig, ax = plt.subplots(figsize=(12, 6))

    # Print comparison table
    print("\nController Performance Comparison")
    print("-" * 108)

    print(
        f"{'Controller':<14}"
        f"{'Kp':>6}"
        f"{'Ki':>6}"
        f"{'Kd':>6}"
        f"{'Rise (s)':>14}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Error (m)':>15}"
    )

    print("-" * 108)

    for name, gains in configurations.items():
        kp, ki, kd = gains

        # Initialize controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
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
            f"{name:<14}"
            f"{kp:>6}"
            f"{ki:>6}"
            f"{kd:>6}"
            f"{rise_text:>14}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['steady_state_error']:>15.6f}"
        )

        # Plot controller response
        ax.plot(
            time,
            position,
            linewidth=2,
            label=f"{name} ({kp}, {ki}, {kd})",
        )

    print("-" * 108)

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

    # Graph formatting
    ax.set_title(
        "Comparison of P, PI, PD and PID Controllers",
        fontsize=14,
        fontweight="bold",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")

    ax.set_xlim(0, 5)
    ax.set_ylim(0, 1.3)

    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "05_comparison.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
