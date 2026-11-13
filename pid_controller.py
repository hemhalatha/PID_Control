"""Discrete parallel PID controller with optional practical safeguards."""


class PIDController:
    """Return the actuator command and retain the individual PID terms."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.01,
                 output_limits=(None, None), anti_windup=False,
                 derivative_on_measurement=True, derivative_filter_tau=0.0):
        if dt <= 0:
            raise ValueError("Sampling interval must be positive.")
        if derivative_filter_tau < 0:
            raise ValueError("Filter time constant cannot be negative.")
        lower, upper = output_limits
        if lower is not None and upper is not None and lower >= upper:
            raise ValueError("Lower output limit must be below upper limit.")

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        self.output_limits = output_limits
        self.anti_windup = anti_windup
        self.derivative_on_measurement = derivative_on_measurement
        self.derivative_filter_tau = float(derivative_filter_tau)
        self.reset()

    def reset(self):
        """Clear all stored controller state."""
        self.integral = 0.0  # Integral contribution, in output units.
        self.previous_error = None
        self.previous_measurement = None
        self.filtered_derivative = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, setpoint, measurement):
        """Calculate one PID update; the caller holds the returned output."""
        error = setpoint - measurement
        self.p_term = self.kp * error

        if self.previous_error is None:
            derivative = 0.0
        elif self.derivative_on_measurement:
            derivative = -(measurement - self.previous_measurement) / self.dt
        else:
            derivative = (error - self.previous_error) / self.dt

        alpha = self.dt / (self.derivative_filter_tau + self.dt)
        self.filtered_derivative += alpha * (derivative - self.filtered_derivative)
        self.d_term = self.kd * self.filtered_derivative

        integral_change = self.ki * error * self.dt
        candidate = self.integral + integral_change
        requested = self.p_term + candidate + self.d_term
        lower, upper = self.output_limits

        worsening = (
            (upper is not None and requested > upper and integral_change > 0)
            or (lower is not None and requested < lower and integral_change < 0)
        )
        if not (self.anti_windup and worsening):
            self.integral = candidate

        self.i_term = self.integral
        output = self.p_term + self.i_term + self.d_term
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        self.previous_error = error
        self.previous_measurement = measurement
        return output
