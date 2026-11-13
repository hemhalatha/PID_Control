
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

    target = 1.0
    duration = 10.0
    dt = 0.01

    kp = 50
    ki = 100
    kd = 5

    noise_amplitude = 0.001
    filter_tau = 0.05

    # Configurations: (noise amplitude, filter time constant)
    configurations = {
        "Ideal PID": (0.0, 0.0),
        "Noisy PID": (noise_amplitude, 0.0),
        "Filtered PID": (noise_amplitude, filter_tau),
    }

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(12, 10), sharex=True
    )

    print("\nSensor Noise and Derivative Filtering")
    print("-" * 85)

    print(
        f"{'Controller':<18}"
        f"{'D Std (N)':>15}"
        f"{'D Peak-Peak (N)':>20}"
        f"{'Force Std (N)':>18}"
        f"{'Pos Std (mm)':>16}"
    )

    print("-" * 85)

    noise_statistics = {}

    for name, (noise, tau) in configurations.items():

        # Configure the PID controller
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            derivative_on_measurement=True,
            derivative_filter_tau=tau,
        )

        # Run simulation with reproducible sensor noise
        results = simulate(
            controller,
            plant,
            duration=duration,
            setpoint=target,
            plant_dt=0.001,
            noise_amplitude=noise,
            seed=42,
        )

        time = results["time"]
        position = results["position"]
        measurement = results["measurement"]
        derivative = results["d_term"]
        force = results["force"]

        # Analyze the steady-state region.
        # Use controller sampling points only.
        sample_interval = round(dt / 0.001)

        sample_indices = np.arange(
            0, len(time), sample_interval
        )

        steady_indices = sample_indices[
            time[sample_indices] >= 5.0
        ]

        steady_derivative = derivative[steady_indices]
        steady_force = force[steady_indices]
        steady_position = position[steady_indices]

        # Calculate noise statistics
        derivative_std = np.std(steady_derivative)

        derivative_peak_to_peak = np.ptp(
            steady_derivative
        )

        force_std = np.std(steady_force)

        position_std = np.std(steady_position) * 1000

        noise_statistics[name] = derivative_std

        print(
            f"{name:<18}"
            f"{derivative_std:>15.6f}"
            f"{derivative_peak_to_peak:>20.6f}"
            f"{force_std:>18.6f}"
            f"{position_std:>16.6f}"
        )

        # Plot true position error in millimetres
        line, = ax1.plot(
            time,
            (position - target) * 1000,
            linewidth=1.5,
            label=name,
        )

        color = line.get_color()

        # Plot noisy sensor measurement for the unfiltered case
        if name == "Noisy PID":
            ax1.plot(
                time,
                (measurement - target) * 1000,
                color="gray",
                alpha=0.35,
                linewidth=0.8,
                label="Noisy measurement",
            )

        # Plot derivative contribution
        ax2.plot(
            time,
            derivative,
            color=color,
            linewidth=1.2,
            label=name,
        )

        # Plot total controller force
        ax3.plot(
            time,
            force,
            color=color,
            linewidth=1.2,
            label=name,
        )

    print("-" * 85)

    # Calculate percentage reduction in derivative noise
    noisy_std = noise_statistics["Noisy PID"]
    filtered_std = noise_statistics["Filtered PID"]

    reduction = (
        (noisy_std - filtered_std) / noisy_std
    ) * 100

    print(
        f"\nReduction in derivative noise: "
        f"{reduction:.2f}%"
    )

    # Position plot
    ax1.axhline(
        0,
        color="black",
        linestyle="--",
        linewidth=1,
        label="Target position",
    )

    ax1.set_title("Position Error and Sensor Noise")
    ax1.set_ylabel("Position deviation (mm)")
    ax1.legend(fontsize=9, loc="upper right")

    # Derivative contribution plot
    ax2.axhline(
        0,
        color="black",
        linestyle="--",
        linewidth=1,
    )

    ax2.set_title("Derivative Force Fluctuations")
    ax2.set_ylabel("Derivative force (N)")
    ax2.legend(fontsize=9, loc="upper right")

    # Controller output plot
    ax3.axhline(
        20,
        color="black",
        linestyle="--",
        linewidth=1,
        label="Required holding force (20 N)",
    )

    ax3.set_title("Controller Output Force")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Force (N)")
    ax3.legend(fontsize=9, loc="upper right")

    # Display the steady-state region
    for ax in (ax1, ax2, ax3):
        ax.set_xlim(5, 7)
        ax.grid(True, alpha=0.3)

    # Set axis ranges to highlight small fluctuations
    ax1.set_ylim(-1.5, 1.5)
    ax2.set_ylim(-1.2, 1.2)
    ax3.set_ylim(18.5, 21.5)

    fig.suptitle(
        "Effect of Sensor Noise and Derivative Filtering",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "07_sensor_noise.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
