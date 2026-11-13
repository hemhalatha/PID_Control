"""Target-relative step response measures."""

import numpy as np


def first_crossing(time, response, threshold):
    """Interpolate the first upward crossing; return NaN if not reached."""
    indices = np.flatnonzero(response >= threshold)
    if not len(indices):
        return np.nan
    k = indices[0]
    if k == 0:
        return time[0]
    return (time[k - 1] + (threshold - response[k - 1])
            * (time[k] - time[k - 1]) / (response[k] - response[k - 1]))


def calculate_metrics(time, position, setpoint=1.0, tolerance=0.02):
    """Compute metrics relative to a constant commanded step.

    Final error is estimated from the mean of the last 5% of samples
    (at least five), matching the original report's calculation.
    Settling is confirmed only within the observed time interval.
    """
    time = np.asarray(time, dtype=float)
    position = np.asarray(position, dtype=float)
    if time.ndim != 1 or position.ndim != 1 or len(time) != len(position) or len(time) < 2:
        raise ValueError("Time and position must be equal-length 1-D arrays.")
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(position)):
        raise ValueError("Samples must be finite.")
    if np.any(np.diff(time) <= 0) or tolerance <= 0:
        raise ValueError("Invalid time sequence or tolerance.")
    step = setpoint - position[0]
    if step == 0:
        raise ValueError("The target must differ from the initial position.")
    response = (position - position[0]) / step
    t10 = first_crossing(time, response, 0.1)
    t90 = first_crossing(time, response, 0.9)
    rise = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan
    peak_index = np.argmax(response)
    overshoot = max(0.0, (response[peak_index] - 1.0) * 100)
    outside = np.flatnonzero(np.abs(position - setpoint) > tolerance * abs(step))
    if not len(outside):
        settling = time[0]
    elif outside[-1] < len(time) - 1:
        settling = time[outside[-1] + 1]
    else:
        settling = np.nan
    error = setpoint - np.mean(position[-max(5, len(position) // 20):])
    return dict(rise_time=rise, overshoot=overshoot, settling_time=settling,
                steady_state_error=error, final_position=position[-1],
                peak_time=time[peak_index], peak_position=position[peak_index],
                t10=t10, t90=t90)


def response_metrics(run):
    """Calculate metrics for a run with a constant setpoint."""
    target = run["setpoint"][0]
    if not np.allclose(run["setpoint"], target):
        raise ValueError("Step metrics require a constant target after t=0.")
    return calculate_metrics(run["time"], run["position"], target)


def format_number(value, digits=4):
    return f"{value:.{digits}f}" if np.isfinite(value) else "N/A"


def print_metrics(name, values):
    print(f"{name}: rise={format_number(values['rise_time'])} s, "
          f"overshoot={format_number(values['overshoot'], 2)}%, "
          f"settling={format_number(values['settling_time'])} s, "
          f"final error estimate={format_number(values['steady_state_error'], 6)}")
