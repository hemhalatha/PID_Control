import numpy as np


class MassSpringDamper:
    """Simulate a mass attached to a spring and damper."""

    def __init__(self, mass=1.0, damping=10.0, spring=20.0):
        if mass <= 0:
            raise ValueError("Mass must be positive.")

        if damping < 0 or spring < 0:
            raise ValueError(
                "Damping and spring stiffness cannot be negative."
            )

        self.mass = mass
        self.damping = damping
        self.spring = spring

        self.reset()

    def reset(self, position=0.0, velocity=0.0):
        """Reset the initial position and velocity."""

        self.position = position
        self.velocity = velocity

    def acceleration(self, position, velocity, force):
        """Calculate acceleration using the equation of motion."""

        return (
            force
            - self.damping * velocity
            - self.spring * position
        ) / self.mass

    def _derivatives(self, state, force):
        """Return the derivatives of position and velocity."""

        position, velocity = state

        acceleration = self.acceleration(
            position, velocity, force
        )

        return np.array([velocity, acceleration])

    def step(self, force, dt):
        """Advance the plant by one time step using RK4."""

        if dt <= 0:
            raise ValueError("Time step must be positive.")

        state = np.array([self.position, self.velocity])

        k1 = self._derivatives(state, force)

        k2 = self._derivatives(
            state + 0.5 * dt * k1, force
        )

        k3 = self._derivatives(
            state + 0.5 * dt * k2, force
        )

        k4 = self._derivatives(
            state + dt * k3, force
        )

        state = state + (dt / 6.0) * (
            k1 + 2 * k2 + 2 * k3 + k4
        )

        self.position, self.velocity = state

        return self.position, self.velocity


def simulate(
    controller,
    plant,
    duration=10.0,
    setpoint=1.0,
    plant_dt=0.001,
    noise_amplitude=0.0,
    seed=42,
    initial_position=0.0,
    initial_velocity=0.0,
):
    """Run a closed-loop simulation and return recorded data."""

    if duration <= 0 or plant_dt <= 0:
        raise ValueError("Duration and time step must be positive.")

    if noise_amplitude < 0:
        raise ValueError("Noise amplitude cannot be negative.")

    controller_dt = controller.dt

    # Ensure controller updates align with plant integration steps.
    update_interval = round(controller_dt / plant_dt)

    if update_interval < 1 or not np.isclose(
        update_interval * plant_dt, controller_dt
    ):
        raise ValueError(
            "Controller time step must be a positive multiple "
            "of the plant time step."
        )

    steps = round(duration / plant_dt)

    if not np.isclose(steps * plant_dt, duration):
        raise ValueError(
            "Duration must be a multiple of the plant time step."
        )

    time = np.arange(steps + 1) * plant_dt

    # Reset the controller and plant before every experiment.
    controller.reset()
    plant.reset(initial_position, initial_velocity)

    # Fixed seed makes noisy experiments reproducible.
    rng = np.random.default_rng(seed)

    results = {
        "time": time,
        "setpoint": np.zeros(steps + 1),
        "position": np.zeros(steps + 1),
        "velocity": np.zeros(steps + 1),
        "measurement": np.zeros(steps + 1),
        "error": np.zeros(steps + 1),
        "force": np.zeros(steps + 1),
        "p_term": np.zeros(steps + 1),
        "i_term": np.zeros(steps + 1),
        "d_term": np.zeros(steps + 1),
    }

    force = 0.0
    measurement = initial_position

    for i, t in enumerate(time):

        # Setpoint can be a constant or a function of time.
        target = setpoint(t) if callable(setpoint) else setpoint

        # Update the controller only at its sampling interval.
        if i % update_interval == 0:

            noise = rng.uniform(
                -noise_amplitude, noise_amplitude
            )

            measurement = plant.position + noise

            force = controller.update(target, measurement)

        # Record the current state before advancing the plant.
        results["setpoint"][i] = target
        results["position"][i] = plant.position
        results["velocity"][i] = plant.velocity
        results["measurement"][i] = measurement
        results["error"][i] = target - plant.position
        results["force"][i] = force

        results["p_term"][i] = controller.p_term
        results["i_term"][i] = controller.i_term
        results["d_term"][i] = controller.d_term

        # Hold the latest controller force between updates.
        if i < steps:
            plant.step(force, plant_dt)

    return results
