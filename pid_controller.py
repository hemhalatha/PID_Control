
class PIDController:
    """Discrete PID controller with optional practical improvements."""

    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        dt=0.01,
        output_limits=(None, None),
        anti_windup=False,
        derivative_on_measurement=False,
        derivative_filter_tau=0.0,
    ):
        if dt <= 0:
            raise ValueError("Sampling time must be positive.")

        if derivative_filter_tau < 0:
            raise ValueError("Filter time constant cannot be negative.")

        lower, upper = output_limits

        if lower is not None and upper is not None:
            if lower >= upper:
                raise ValueError("Lower limit must be less than upper limit.")

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.output_limits = output_limits
        self.anti_windup = anti_windup
        self.derivative_on_measurement = derivative_on_measurement
        self.derivative_filter_tau = derivative_filter_tau

        self.reset()

    def reset(self):
        """Reset the controller before starting a new simulation."""

        self.previous_error = None
        self.previous_measurement = None

        self.integral = 0.0
        self.filtered_derivative = 0.0

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def _clamp(self, value):
        """Restrict a value to the actuator output limits."""

        lower, upper = self.output_limits

        if lower is not None:
            value = max(lower, value)

        if upper is not None:
            value = min(upper, value)

        return value

    def update(self, setpoint, measurement):
        """Calculate the control output for one time step."""

        error = setpoint - measurement

        # Proportional term
        self.p_term = self.kp * error

        # Derivative term
        if self.previous_error is None:
            derivative = 0.0

        elif self.derivative_on_measurement:
            derivative = -(
                measurement - self.previous_measurement
            ) / self.dt

        else:
            derivative = (
                error - self.previous_error
            ) / self.dt

        # Optional low-pass filter for the derivative
        tau = self.derivative_filter_tau

        if tau > 0:
            alpha = tau / (tau + self.dt)

            self.filtered_derivative = (
                alpha * self.filtered_derivative
                + (1 - alpha) * derivative
            )
        else:
            self.filtered_derivative = derivative

        self.d_term = self.kd * self.filtered_derivative

        # Calculate a candidate integral value
        integral_change = error * self.dt
        new_integral = self.integral + integral_change

        # Calculate output using the candidate integral
        tentative_output = (
            self.p_term
            + self.ki * new_integral
            + self.d_term
        )

        # Conditional integration to prevent integral windup
        if self.anti_windup:
            lower, upper = self.output_limits

            above_limit = (
                upper is not None and tentative_output > upper
            )

            below_limit = (
                lower is not None and tentative_output < lower
            )

            integral_output_change = self.ki * integral_change

            if above_limit and integral_output_change > 0:
                pass

            elif below_limit and integral_output_change < 0:
                pass

            else:
                self.integral = new_integral

        else:
            self.integral = new_integral

        self.i_term = self.ki * self.integral

        # Combine the three terms and apply actuator limits
        output = self.p_term + self.i_term + self.d_term
        output = self._clamp(output)

        # Store values for the next time step
        self.previous_error = error
        self.previous_measurement = measurement

        return output