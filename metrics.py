
import numpy as np


def calculate_metrics(time, position, setpoint=1.0, tolerance=0.02):
    """Calculate performance metrics for a step response."""

    time = np.asarray(time, dtype=float)
    position = np.asarray(position, dtype=float)

    if time.ndim != 1 or position.ndim != 1:
        raise ValueError("Time and position must be one-dimensional.")

    if len(time) != len(position) or len(time) < 2:
        raise ValueError(
            "Time and position must have equal lengths "
            "with at least two samples."
        )

    if np.any(np.diff(time) <= 0):
        raise ValueError("Time values must be strictly increasing.")

    if not np.all(np.isfinite(position)):
        raise ValueError("Position contains invalid values.")

    if tolerance <= 0:
        raise ValueError("Tolerance must be positive.")

    initial_position = position[0]
    step_size = setpoint - initial_position

    if step_size == 0:
        raise ValueError("Setpoint must differ from initial position.")

    # Normalize the response to handle positive and negative steps.
    response = (position - initial_position) / step_size

    # 1. Rise time: time from 10% to 90% of the target change.
    t10 = first_crossing(time, response, 0.1)
    t90 = first_crossing(time, response, 0.9)

    rise_time = None

    if t10 is not None and t90 is not None:
        rise_time = t90 - t10

    # 2. Overshoot: maximum response above the target.
    peak_index = np.argmax(response)

    overshoot = max(
        0.0, (response[peak_index] - 1.0) * 100
    )

    peak_time = time[peak_index]
    peak_position = position[peak_index]

    # 3. Settling time: last entry into the tolerance band.
    band = tolerance * abs(step_size)

    outside_band = np.where(
        np.abs(position - setpoint) > band
    )[0]

    settling_time = None

    if len(outside_band) == 0:
        settling_time = time[0]

    elif outside_band[-1] < len(time) - 1:
        settling_time = time[outside_band[-1] + 1]

    # 4. Steady-state error: target minus final position.
    final_position = position[-1]

    steady_state_error = setpoint - final_position

    steady_state_error_percent = (
        abs(steady_state_error) / abs(step_size)
    ) * 100

    return {
        "rise_time": rise_time,
        "overshoot": overshoot,
        "settling_time": settling_time,
        "steady_state_error": steady_state_error,
        "steady_state_error_percent": steady_state_error_percent,
        "final_position": final_position,
        "peak_position": peak_position,
        "peak_time": peak_time,
        "t10": t10,
        "t90": t90,
    }


def first_crossing(time, response, threshold):
    """Find the first time a response reaches a threshold."""

    indices = np.where(response >= threshold)[0]

    if len(indices) == 0:
        return None

    index = indices[0]

    if index == 0:
        return time[0]

    # Linear interpolation between adjacent samples.
    t1 = time[index - 1]
    t2 = time[index]

    y1 = response[index - 1]
    y2 = response[index]

    if y1 == y2:
        return t2

    return t1 + (threshold - y1) * (t2 - t1) / (y2 - y1)


def print_metrics(name, metrics):
    """Print performance metrics in a readable format."""

    def format_time(value):
        if value is None:
            return "Not reached"

        return f"{value:.3f} s"

    print(f"\n{name}")
    print("-" * 45)

    print(
        f"Rise time          : "
        f"{format_time(metrics['rise_time'])}"
    )

    print(
        f"Overshoot          : "
        f"{metrics['overshoot']:.2f}%"
    )

    print(
        f"Settling time      : "
        f"{format_time(metrics['settling_time'])}"
    )

    print(
        f"Steady-state error : "
        f"{metrics['steady_state_error']:.6f} m"
    )

    print(
        f"Final position     : "
        f"{metrics['final_position']:.6f} m"
    )