"""Physical plant models and sampled closed-loop simulation."""

import numpy as np

from pid_controller import PIDController


class MassSpringDamper:
    """Linear plant with state [position, velocity]."""

    def __init__(self, mass=1.0, damping=10.0, spring=20.0):
        if mass <= 0 or damping < 0 or spring < 0:
            raise ValueError("Invalid mass, damping or spring stiffness.")
        self.mass = float(mass)
        self.damping = float(damping)
        self.spring = float(spring)
        self.state = np.zeros(2)

    def reset(self, position=0.0, velocity=0.0):
        self.state = np.array([position, velocity], dtype=float)

    @property
    def position(self):
        return self.state[0]

    @property
    def velocity(self):
        return self.state[1]

    def derivatives(self, state, force):
        x, v = state
        return np.array([
            v, (force - self.damping * v - self.spring * x) / self.mass
        ])

    def step(self, force, dt):
        self.state = rk4(self.derivatives, self.state, force, dt)
        return self.position, self.velocity


class ThirdOrderPlant:
    """Three first-order lags in series: G(s) = 1/(s+1)^3."""

    def __init__(self):
        self.state = np.zeros(3)

    def reset(self, position=0.0, velocity=0.0):
        self.state = np.array([position + velocity, position + velocity, position],
                              dtype=float)

    @property
    def position(self):
        return self.state[2]

    @property
    def velocity(self):
        return self.state[1] - self.state[2]

    @staticmethod
    def derivatives(state, control):
        a, b, c = state
        return np.array([-a + control, -b + a, -c + b])

    def step(self, force, dt):
        self.state = rk4(self.derivatives, self.state, force, dt)
        return self.position, self.velocity


def rk4(derivatives, state, control, dt):
    """Advance a state by one fourth-order Runge-Kutta step."""
    k1 = derivatives(state, control)
    k2 = derivatives(state + dt * k1 / 2, control)
    k3 = derivatives(state + dt * k2 / 2, control)
    k4 = derivatives(state + dt * k3, control)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def simulate(controller, plant, duration=8.0, setpoint=1.0, plant_dt=0.002,
             noise_amplitude=0.0, seed=10, initial_position=0.0,
             initial_velocity=0.0):
    """Record at controller updates; integrate the plant with finer RK4 steps.

    A callable setpoint is evaluated at each controller update. The final
    recorded time is the nearest whole controller interval to duration.
    """
    if duration <= 0 or plant_dt <= 0 or noise_amplitude < 0:
        raise ValueError("Invalid simulation duration, step or noise amplitude.")
    dt = controller.dt
    count = int(round(duration / dt)) + 1
    if count < 2:
        raise ValueError("Duration must include at least two samples.")
    times = np.arange(count) * dt
    steps = max(1, int(np.ceil(dt / plant_dt)))
    h = dt / steps
    rng = np.random.default_rng(seed)
    controller.reset()
    plant.reset(initial_position, initial_velocity)

    keys = ("position", "velocity", "force", "setpoint", "p_term", "i_term",
            "d_term", "measurement", "error")
    results = {key: np.zeros(count) for key in keys}
    results["time"] = times

    for j, t in enumerate(times):
        target = setpoint(t) if callable(setpoint) else setpoint
        value = plant.position
        measurement = value + rng.uniform(-noise_amplitude, noise_amplitude)
        force = controller.update(target, measurement)

        results["position"][j] = value
        results["velocity"][j] = plant.velocity
        results["force"][j] = force
        results["setpoint"][j] = target
        results["p_term"][j] = controller.p_term
        results["i_term"][j] = controller.i_term
        results["d_term"][j] = controller.d_term
        results["measurement"][j] = measurement
        results["error"][j] = target - value

        if j == count - 1:
            break
        for _ in range(steps):
            plant.step(force, h)
    return results


def run_response(kp, ki=0.0, kd=0.0, duration=8.0, dt=0.01,
                 plant=None, output_limit=None, anti_windup=False,
                 derivative_on_measurement=True, filter_tau=0.0, noise=0.0,
                 seed=10, target=1.0, plant_dt=0.002):
    """Convenience wrapper for experiments with the original report settings."""
    limits = (None, None) if output_limit is None else (-output_limit, output_limit)
    controller = PIDController(kp, ki, kd, dt, limits, anti_windup,
                               derivative_on_measurement, filter_tau)
    if plant is None:
        plant = MassSpringDamper()
    return simulate(controller, plant, duration=duration, setpoint=target,
                    plant_dt=plant_dt, noise_amplitude=noise, seed=seed)
