"""Numerical regression tests for the original report configurations."""

import unittest

import numpy as np

from experiments.exp10_ziegler_nichols import (
    sampled_ultimate_values,
    ziegler_nichols_gains,
)
from metrics import calculate_metrics, response_metrics
from pid_controller import PIDController
from plant import ThirdOrderPlant, run_response


class ControllerTests(unittest.TestCase):
    def test_proportional_and_integral(self):
        pid = PIDController(20, 10, dt=0.1)
        self.assertAlmostEqual(pid.update(1, 0.5), 10.5)
        self.assertAlmostEqual(pid.i_term, 0.5)
        pid.reset()
        self.assertEqual(pid.i_term, 0.0)

    def test_saturation_and_anti_windup(self):
        pid = PIDController(70, 180, 12, output_limits=(-30, 30), anti_windup=True)
        self.assertEqual(pid.update(1, 0), 30)
        self.assertEqual(pid.i_term, 0)

    def test_derivative_kick(self):
        for on_measurement, expected in ((False, 1500), (True, 0)):
            pid = PIDController(100, 100, 15, derivative_on_measurement=on_measurement)
            pid.update(0, 0)
            pid.update(1, 0)
            self.assertAlmostEqual(pid.d_term, expected)

    def test_derivative_filter(self):
        pid = PIDController(kd=15, derivative_filter_tau=0.05)
        pid.update(0, 0)
        pid.update(0, 0.001)
        self.assertAlmostEqual(pid.d_term, -0.25)


class SimulationTests(unittest.TestCase):
    def test_proportional_equilibria(self):
        for kp in (20, 50, 100, 200):
            run = run_response(kp, duration=5)
            self.assertAlmostEqual(run['position'][-1], kp / (kp + 20), places=5)

    def test_original_pid_metrics(self):
        met = response_metrics(run_response(100, 100, 15, duration=8))
        self.assertAlmostEqual(met['rise_time'], 0.4043, places=4)
        self.assertAlmostEqual(met['overshoot'], 1.73, places=2)
        self.assertAlmostEqual(met['settling_time'], 0.65, places=6)
        self.assertLess(abs(met['steady_state_error']), 0.0001)

    def test_original_step_response(self):
        met = response_metrics(run_response(100, 120, 20, duration=6))
        self.assertAlmostEqual(met['rise_time'], 0.4386, places=4)
        self.assertAlmostEqual(met['overshoot'], 5.89, places=2)
        self.assertAlmostEqual(met['settling_time'], 2.23, places=6)

    def test_noise_is_reproducible(self):
        a = run_response(100, 100, 15, duration=2, noise=0.001)
        b = run_response(100, 100, 15, duration=2, noise=0.001)
        np.testing.assert_array_equal(a['measurement'], b['measurement'])

    def test_tolerance_parameter(self):
        t = [0, 1, 2, 3]
        y = [0, 0.9, 0.97, 1.0]
        self.assertEqual(calculate_metrics(t, y, tolerance=0.05)['settling_time'], 2)
        self.assertEqual(calculate_metrics(t, y, tolerance=0.02)['settling_time'], 3)

    def test_sampled_ultimate_gain_and_period(self):
        ku, tu = sampled_ultimate_values()
        self.assertAlmostEqual(ku, 7.882159, places=5)
        self.assertAlmostEqual(tu, 3.652, places=3)

    def test_sampled_zn_retuning(self):
        ku, tu = sampled_ultimate_values()
        gains = ziegler_nichols_gains(ku, tu)["PID"]
        baseline = response_metrics(run_response(
            *gains, plant=ThirdOrderPlant(), duration=18,
        ))
        improved = response_metrics(run_response(
            3, 1, 1.5, plant=ThirdOrderPlant(), duration=18,
        ))
        self.assertAlmostEqual(baseline["overshoot"], 52.19, places=2)
        self.assertAlmostEqual(improved["overshoot"], 21.05, places=2)
        self.assertLess(improved["settling_time"], baseline["settling_time"])
        self.assertLess(abs(improved["steady_state_error"]), 0.001)

    def test_steady_state_derivative_noise_reduction(self):
        deviations = []
        for tau in (0.0, 0.05):
            run = run_response(
                100, 100, 15, duration=8, noise=0.001, filter_tau=tau,
            )
            derivative = run["d_term"][run["time"] >= 5]
            deviations.append(np.std(derivative))
        self.assertLess(deviations[1], 0.2 * deviations[0])

    def test_combined_practical_controller(self):
        run = run_response(
            100, 100, 15, duration=8, dt=0.01,
            output_limit=30, anti_windup=True,
            derivative_on_measurement=True, filter_tau=0.05,
            noise=0.001, seed=10,
        )
        for signal in run.values():
            self.assertTrue(np.all(np.isfinite(signal)))
        self.assertLessEqual(np.max(np.abs(run["force"])), 30)
        met = response_metrics(run)
        self.assertLess(met["overshoot"], 1.0)
        self.assertLess(met["settling_time"], 2.0)
        self.assertLess(abs(met["steady_state_error"]), 0.001)
        self.assertLess(np.max(np.abs(run["position"][-100:] - 1)), 0.001)


if __name__ == '__main__':
    unittest.main()
