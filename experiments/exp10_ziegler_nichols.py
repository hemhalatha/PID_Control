
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pid_controller import PIDController
from plant import simulate
from metrics import calculate_metrics


class ThirdOrderPlant:
    """Plant with transfer function G(s) = 1 / (s + 1)^3."""

    def __init__(self):
        self.reset()

    def reset(self, position=0.0, velocity=0.0):
        """Reset the three internal states."""

        self.state = np.array(
            [position + velocity, position + velocity, position],
            dtype=float,
        )

    @property
    def position(self):
        """Return the plant output."""

        return self.state[2]

    @property
    def velocity(self):
        """Return the rate of change of the output."""

        return self.state[1] - self.state[2]

    @staticmethod
    def derivatives(state, control):
        """Calculate the derivatives of the three states."""

        x1, x2, x3 = state

        return np.array([
            control - x1,
            x1 - x2,
            x2 - x3,
        ])

    @classmethod
    def advance(cls, state, control, dt):
        """Advance the states using fourth-order Runge-Kutta."""

        k1 = cls.derivatives(state, control)

        k2 = cls.derivatives(
            state + 0.5 * dt * k1, control
        )

        k3 = cls.derivatives(
            state + 0.5 * dt * k2, control
        )

        k4 = cls.derivatives(
            state + dt * k3, control
        )

        return state + (dt / 6.0) * (
            k1 + 2 * k2 + 2 * k3 + k4
        )

    def step(self, force, dt):
        """Advance the plant by one time step."""

        self.state = self.advance(
            self.state, force, dt
        )

        return self.position, self.velocity


def find_ultimate_gain(dt):
    """Find the proportional gain at marginal stability."""

    # Construct the discrete plant state-transition matrix.
    identity = np.eye(3)

    a_matrix = np.column_stack([
        ThirdOrderPlant.advance(
            identity[:, i], 0.0, dt
        )
        for i in range(3)
    ])

    # Calculate the effect of one unit of control input.
    b_vector = ThirdOrderPlant.advance(
        np.zeros(3), 1.0, dt
    )

    # The output is the third state.
    c_vector = np.array([0.0, 0.0, 1.0])

    def spectral_radius(kp):
        """Return the largest closed-loop eigenvalue magnitude."""

        closed_loop = (
            a_matrix
            - kp * np.outer(b_vector, c_vector)
        )

        eigenvalues = np.linalg.eigvals(closed_loop)

        return np.max(np.abs(eigenvalues))

    # Search between a stable and an unstable gain.
    lower = 0.0
    upper = 12.0

    if spectral_radius(upper) < 1.0:
        raise ValueError(
            "Increase the upper gain search limit."
        )

    # Binary search for the stability boundary.
    for _ in range(50):
        middle = (lower + upper) / 2.0

        if spectral_radius(middle) < 1.0:
            lower = middle
        else:
            upper = middle

    return (lower + upper) / 2.0


def measure_period(time, position):
    """Measure the oscillation period from successive peaks."""

    peak_indices = np.where(
        (position[1:-1] > position[:-2])
        & (position[1:-1] >= position[2:])
    )[0] + 1

    # Ignore the initial transient.
    peak_indices = peak_indices[
        time[peak_indices] >= 10.0
    ]

    if len(peak_indices) < 2:
        raise ValueError(
            "Not enough peaks to measure the period."
        )

    # Average the last five complete oscillation periods.
    peak_times = time[peak_indices[-6:]]

    periods = np.diff(peak_times)

    return np.mean(periods), peak_times


def ziegler_nichols_gains(ku, tu):
    """Calculate classical P, PI and PID gains."""

    # Proportional controller
    p_kp = 0.5 * ku

    # PI controller
    pi_kp = 0.45 * ku
    pi_ti = tu / 1.2
    pi_ki = pi_kp / pi_ti

    # PID controller
    pid_kp = 0.6 * ku
    pid_ti = tu / 2.0
    pid_td = tu / 8.0

    pid_ki = pid_kp / pid_ti
    pid_kd = pid_kp * pid_td

    return {
        "P": (p_kp, 0.0, 0.0),
        "PI": (pi_kp, pi_ki, 0.0),
        "PID": (pid_kp, pid_ki, pid_kd),
    }


def main():
    plant = ThirdOrderPlant()

    dt = 0.01
    target = 1.0

    # Find the ultimate gain of the sampled system.
    ku = find_ultimate_gain(dt)

    # Simulate proportional control at the ultimate gain.
    ultimate_controller = PIDController(
        kp=ku,
        ki=0.0,
        kd=0.0,
        dt=dt,
    )

    ultimate_results = simulate(
        ultimate_controller,
        plant,
        duration=50.0,
        setpoint=target,
        plant_dt=dt,
    )

    # Measure the oscillation period.
    tu, peak_times = measure_period(
        ultimate_results["time"],
        ultimate_results["position"],
    )

    # Theoretical continuous-time values.
    theoretical_ku = 8.0
    theoretical_tu = 2 * np.pi / np.sqrt(3)

    print("\nUltimate Gain and Oscillation Period")
    print("-" * 50)

    print(f"Theoretical Ku : {theoretical_ku:.6f}")
    print(f"Measured Ku    : {ku:.6f}")

    print(f"Theoretical Tu : {theoretical_tu:.6f} s")
    print(f"Measured Tu    : {tu:.6f} s")

    print("-" * 50)

    # Calculate Ziegler-Nichols gains.
    configurations = ziegler_nichols_gains(ku, tu)

    print("\nZiegler-Nichols Tuning Results")
    print("-" * 48)

    print(
        f"{'Controller':<14}"
        f"{'Kp':>10}"
        f"{'Ki':>10}"
        f"{'Kd':>10}"
    )

    print("-" * 48)

    for name, (kp, ki, kd) in configurations.items():
        print(
            f"{name:<14}"
            f"{kp:>10.4f}"
            f"{ki:>10.4f}"
            f"{kd:>10.4f}"
        )

    print("-" * 48)

    fig, axes = plt.subplots(
        2, 2, figsize=(14, 10)
    )

    ax1, ax2 = axes[0]
    ax3, ax4 = axes[1]

    # --------------------------------------------------
    # Plot 1: Increase proportional gain towards Ku.
    # --------------------------------------------------

    kp_values = [4.0, 6.0, ku, 1.05 * ku]

    for kp in kp_values:
        controller = PIDController(
            kp=kp,
            dt=dt,
        )

        results = simulate(
            controller,
            plant,
            duration=25.0,
            setpoint=target,
            plant_dt=dt,
        )

        label = f"Kp = {kp:.3f}"

        if np.isclose(kp, ku):
            label += " (Ku)"

        ax1.plot(
            results["time"],
            results["position"],
            linewidth=1.6,
            label=label,
        )

    ax1.axhline(
        target,
        color="black",
        linestyle="--",
        label="Setpoint",
    )

    ax1.set_title("Increasing Proportional Gain")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Output (normalized units)")
    ax1.legend(fontsize=9)

    # --------------------------------------------------
    # Plot 2: Sustained oscillations and period.
    # --------------------------------------------------

    time = ultimate_results["time"]
    position = ultimate_results["position"]

    ax2.plot(
        time,
        position,
        color="royalblue",
        linewidth=1.8,
        label=f"Ku = {ku:.3f}",
    )

    # Mark two consecutive peaks.
    first_peak = peak_times[-2]
    second_peak = peak_times[-1]

    first_value = np.interp(
        first_peak, time, position
    )

    second_value = np.interp(
        second_peak, time, position
    )

    ax2.scatter(
        [first_peak, second_peak],
        [first_value, second_value],
        color="red",
        zorder=5,
        label="Consecutive peaks",
    )

    annotation_height = max(
        first_value, second_value
    ) + 0.15

    ax2.annotate(
        "",
        xy=(second_peak, annotation_height),
        xytext=(first_peak, annotation_height),
        arrowprops=dict(
            arrowstyle="<->",
            color="black",
        ),
    )

    ax2.text(
        (first_peak + second_peak) / 2,
        annotation_height + 0.05,
        f"Tu = {tu:.3f} s",
        horizontalalignment="center",
        fontsize=10,
    )

    ax2.set_title("Sustained Oscillations at Ultimate Gain")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Output (normalized units)")

    ax2.set_xlim(
        first_peak - 4.0,
        second_peak + 0.4,
    )

    ax2.legend(fontsize=9)

    # --------------------------------------------------
    # Plot 3: Ziegler-Nichols controller responses.
    # --------------------------------------------------

    print("\nClosed-Loop Performance")
    print("-" * 80)

    print(
        f"{'Controller':<14}"
        f"{'Rise (s)':>12}"
        f"{'Overshoot (%)':>17}"
        f"{'Settling (s)':>16}"
        f"{'Final Error':>16}"
    )

    print("-" * 80)

    for name, (kp, ki, kd) in configurations.items():
        controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            derivative_on_measurement=True,
        )

        results = simulate(
            controller,
            plant,
            duration=30.0,
            setpoint=target,
            plant_dt=dt,
        )

        time = results["time"]
        position = results["position"]

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
            f"{rise_text:>12}"
            f"{metrics['overshoot']:>17.2f}"
            f"{settling_text:>16}"
            f"{metrics['steady_state_error']:>16.6f}"
        )

        line, = ax3.plot(
            time,
            position,
            linewidth=2,
            label=f"ZN {name}",
        )

        ax4.plot(
            time,
            results["force"],
            color=line.get_color(),
            linewidth=1.6,
            label=f"ZN {name}",
        )

    print("-" * 80)

    ax3.axhline(
        target,
        color="black",
        linestyle="--",
        label="Setpoint",
    )

    ax3.axhspan(
        0.98,
        1.02,
        color="green",
        alpha=0.1,
        label="±2% settling band",
    )

    ax3.set_title("Ziegler-Nichols Tuned Responses")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Output (normalized units)")
    ax3.set_xlim(0, 15)
    ax3.legend(fontsize=9)

    # --------------------------------------------------
    # Plot 4: Controller output.
    # --------------------------------------------------

    ax4.axhline(
        1.0,
        color="black",
        linestyle="--",
        label="Equilibrium input",
    )

    ax4.set_title("Controller Output")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Control input (normalized units)")
    ax4.set_xlim(0, 15)
    ax4.legend(fontsize=9)

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        "Ziegler-Nichols Tuning of G(s) = 1/(s + 1)^3",
        fontsize=14,
        fontweight="bold",
    )

    fig.tight_layout()

    # Save the graph.
    project_dir = Path(__file__).resolve().parents[1]
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    output_path = graph_dir / "10_ziegler_nichols.png"

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"\nGraph saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
