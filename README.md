
# PID Control: Simulation and Analysis

A Python-based study of Proportional-Integral-Derivative (PID)
control using numerical simulation, mathematical analysis, and
experimental comparison.

This project demonstrates the fundamentals of PID control,
controller tuning, performance evaluation, and practical control
system challenges using a mass-spring-damper system.

The PID controller and numerical integration methods are
implemented from scratch using Python and NumPy, without
ready-made control system libraries.

## Objectives

- Understand proportional, integral, and derivative control.
- Compare P, PI, PD, and PID controllers.
- Evaluate controller performance using step response metrics.
- Investigate actuator saturation and integral windup.
- Analyze sensor noise and derivative filtering.
- Demonstrate derivative kick prevention.
- Study the effects of controller sampling time.
- Apply and improve Ziegler-Nichols PID tuning.

## Project Structure

```text
pid-control/
├── pid_controller.py
├── plant.py
├── metrics.py
├── run_experiments.py
├── requirements.txt
├── README.md
├── experiments/
│   ├── __init__.py
│   ├── exp01_step_response.py
│   ├── exp02_proportional.py
│   ├── exp03_integral.py
│   ├── exp04_derivative.py
│   ├── exp05_comparison.py
│   ├── exp06_windup.py
│   ├── exp07_sensor_noise.py
│   ├── exp08_derivative_kick.py
│   ├── exp09_sampling_time.py
│   ├── exp10_ziegler_nichols.py
│   └── exp11_improved_tuning.py
└── graphs/
```

## Requirements

- Python 3.10 or later
- NumPy
- Matplotlib

No additional control system libraries are required.

## Installation

Clone the repository and navigate to the project directory.

```bash
git clone <repository-url>
cd pid-control
```

Create a virtual environment.

```bash
python -m venv .venv
```

Activate the virtual environment.

**Windows PowerShell**

```powershell
.\.venv\Scripts\Activate.ps1
```

**Linux / macOS**

```bash
source .venv/bin/activate
```

Install the dependencies.

```bash
python -m pip install -r requirements.txt
```

## Running the Experiments

### Run All Experiments

Execute the following command from the project root:

```bash
python run_experiments.py
```

This command runs all 11 experiments, prints their numerical
results, and saves the generated figures in the `graphs/`
directory.

The runner uses a non-interactive Matplotlib backend, so the
experiments execute without opening individual plot windows.

### Run Individual Experiments

Each experiment can also be executed independently.

```bash
python -m experiments.exp01_step_response
```

For example, to investigate integral windup:

```bash
python -m experiments.exp06_windup
```

When executed individually, an experiment saves its figure and
displays it using Matplotlib.

## System Model

### Mass-Spring-Damper System

The primary plant is a block attached to a linear spring
and a viscous damper.

The equation of motion is:

$$
m\ddot{x} + b\dot{x} + kx = F
$$

Rearranging for acceleration:

$$
\ddot{x} = \frac{F-b\dot{x}-kx}{m}
$$

The plant parameters are:

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Mass | m | 1 kg |
| Damping coefficient | b | 10 N·s/m |
| Spring stiffness | k | 20 N/m |
| Target position | r | 1 m |
| Initial position | x(0) | 0 m |
| Initial velocity | v(0) | 0 m/s |

The corresponding transfer function is:

$$
G(s)=\frac{X(s)}{F(s)}
=\frac{1}{s^2+10s+20}
$$

At the target position, the block requires a constant holding
force to balance the spring force:

$$
F=kx=20(1)=20\text{ N}
$$

This explains why proportional control alone cannot eliminate
steady-state error in this system.

## PID Controller

The continuous-time PID control equation is:

$$
u(t)=K_pe(t)+K_i\int_0^t e(\tau)d\tau
+K_d\frac{de(t)}{dt}
$$

where:

$$
e(t)=r(t)-x(t)
$$

The controller consists of three terms:

| Term | Function |
|------|----------|
| Proportional | Responds to the current error |
| Integral | Accumulates past error |
| Derivative | Responds to the rate of change of error |

The controller uses a discrete-time implementation:

$$
e_k=r_k-x_k
$$

$$
P_k=K_pe_k
$$

$$
I_k=I_{k-1}+K_ie_k\Delta t
$$

$$
D_k=K_d\frac{e_k-e_{k-1}}{\Delta t}
$$

$$
u_k=P_k+I_k+D_k
$$

The reusable controller in `pid_controller.py` also supports:

- Configurable output limits.
- Conditional integration for anti-windup.
- Derivative on measurement.
- First-order derivative filtering.
- Configurable sampling intervals.
- Independent access to P, I, and D contributions.
- Controller state reset.

## Numerical Simulation

The mass-spring-damper plant is represented by two
first-order differential equations:

$$
\dot{x}=v
$$

$$
\dot{v}=\frac{F-bv-kx}{m}
$$

The equations are integrated using the fourth-order
Runge-Kutta (RK4) method.

The default simulation intervals for the primary plant are:

| Parameter | Value |
|-----------|-------|
| Controller sampling interval | 0.01 s |
| Plant integration interval | 0.001 s |

The controller output is held constant between consecutive
updates using a zero-order hold.

Separating the controller and plant time steps allows sampling
effects to be studied independently of numerical integration
errors.

Sensor noise experiments use a fixed random seed of 42 to
ensure reproducible results.

## Performance Metrics

The `metrics.py` module calculates the following measures:

| Metric | Definition |
|--------|------------|
| Rise time | Time between reaching 10% and 90% of the commanded step |
| Overshoot | Maximum percentage exceeded beyond the target |
| Settling time | Time after which the response remains within ±2% of the commanded step |
| Steady-state error | Difference between the target and final recorded output |

Rise time and settling time are measured relative to the
commanded target, rather than the final achieved position.

If a controller does not reach the required threshold,
the corresponding metric is reported as N/A.

Steady-state error is estimated from the final simulation
sample. This estimate should only be interpreted as a true
steady-state error when the response has converged.

## Experiments

### 1. Step Response and Performance Metrics

Demonstrates the four performance metrics on a single
annotated PID step response.

```bash
python -m experiments.exp01_step_response
```

![Step Response](graphs/01_step_response.png)

### 2. Proportional Control

Compares proportional gains of 10, 50, 100, and 300.

The theoretical steady-state position is:

$$
x_{ss}=\frac{K_p}{K_p+k}r
$$

The steady-state error is:

$$
e_{ss}=\frac{k}{K_p+k}r
$$

The experiment compares these theoretical values with
the simulation results.

```bash
python -m experiments.exp02_proportional
```

![Proportional Control](graphs/02_proportional.png)

### 3. Integral Control

Compares P and PI controllers while keeping the
proportional gain constant.

Demonstrates how integral action eliminates steady-state
error and how excessive integral gain can introduce
overshoot and oscillations.

```bash
python -m experiments.exp03_integral
```

![Integral Control](graphs/03_integral.png)

### 4. Derivative Control

Investigates how derivative action affects position,
velocity, overshoot, and damping.

For a constant target:

$$
D=-K_d\dot{x}
$$

The derivative term acts as additional damping in the
mass-spring-damper system.

```bash
python -m experiments.exp04_derivative
```

![Derivative Control](graphs/04_derivative.png)

### 5. Controller Comparison

Compares P, PI, PD, PID, and an alternative higher-gain
PID configuration under identical plant conditions.

The experiment reports all four performance metrics.

```bash
python -m experiments.exp05_comparison
```

![Controller Comparison](graphs/05_comparison.png)

### 6. Integral Windup

Introduces an actuator force limit of ±30 N.

Compares PID control with and without conditional
integration to demonstrate integral windup and its
prevention.

```bash
python -m experiments.exp06_windup
```

![Integral Windup](graphs/06_windup.png)

### 7. Sensor Noise

Introduces uniformly distributed sensor noise of ±1 mm.

Compares unfiltered and filtered derivative action using
a first-order low-pass filter with a time constant of
0.05 seconds.

```bash
python -m experiments.exp07_sensor_noise
```

![Sensor Noise](graphs/07_sensor_noise.png)

### 8. Derivative Kick

Changes the setpoint from 0 m to 1 m at t = 1 second.

Compares derivative on error against derivative on
measurement to demonstrate setpoint-induced derivative
kick and its prevention.

```bash
python -m experiments.exp08_derivative_kick
```

![Derivative Kick](graphs/08_derivative_kick.png)

### 9. Sampling Time

Compares controller sampling intervals of 0.01, 0.05,
0.1, 0.2, and 0.5 seconds.

The plant integration interval remains fixed at
0.001 seconds.

Demonstrates the influence of sampling and zero-order
hold on closed-loop stability.

```bash
python -m experiments.exp09_sampling_time
```

![Sampling Time](graphs/09_sampling_time.png)

### 10. Ziegler-Nichols Tuning

Applies the closed-loop Ziegler-Nichols tuning method
to a second system:

$$
G(s)=\frac{1}{(s+1)^3}
$$

The system is implemented as three cascaded first-order
differential equations:

$$
\dot{x}_1=u-x_1
$$

$$
\dot{x}_2=x_1-x_2
$$

$$
\dot{x}_3=x_2-x_3
$$

The output is:

$$
y=x_3
$$

The theoretical continuous-time ultimate gain and period
are:

$$
K_u=8
$$

$$
T_u=\frac{2\pi}{\sqrt{3}}
$$

The experiment determines the sampled-system ultimate
gain, measures the oscillation period, and calculates
the corresponding controller gains.

| Controller | Kp | Ti | Td |
|------------|----|----|----|
| P | 0.5Ku | - | - |
| PI | 0.45Ku | Tu/1.2 | - |
| PID | 0.6Ku | Tu/2 | Tu/8 |

Parallel-form gains are calculated using:

$$
K_i=\frac{K_p}{T_i}
$$

$$
K_d=K_pT_d
$$

```bash
python -m experiments.exp10_ziegler_nichols
```

![Ziegler-Nichols Tuning](graphs/10_ziegler_nichols.png)

### 11. Improved PID Tuning

Compares classical Ziegler-Nichols PID gains with
manually adjusted gains.

The experiment evaluates the trade-off between rise
time, overshoot, settling time, steady-state error,
and control effort.

```bash
python -m experiments.exp11_improved_tuning
```

![Improved PID Tuning](graphs/11_improved_tuning.png)

## Selected Controller Gains

### Mass-Spring-Damper System

The nominal PID configuration used for the practical
control experiments is:

| Gain | Value |
|------|-------|
| Kp | 50 N/m |
| Ki | 100 N/(m·s) |
| Kd | 5 N·s/m |

This configuration provides a balance between tracking
accuracy, overshoot, and response time.

For practical operation, the controller can be
configured with:

- Output limits of -30 N and +30 N.
- Conditional integration.
- Derivative on measurement.
- Derivative filter time constant of 0.05 s.
- Sampling interval of 0.01 s.

The effects of these features are evaluated separately
in the corresponding experiments.

### Third-Order System

The manually improved PID configuration investigated
after Ziegler-Nichols tuning is:

| Gain | Value |
|------|-------|
| Kp | 3.0 |
| Ki | 0.7 |
| Kd | 2.0 |

These gains apply specifically to the normalized
third-order system, not the mass-spring-damper plant.

## Reusing the PID Controller

The controller can be imported into other Python
programs.

```python
from pid_controller import PIDController
from plant import MassSpringDamper, simulate

controller = PIDController(
    kp=50,
    ki=100,
    kd=5,
    dt=0.01,
    output_limits=(-30, 30),
    anti_windup=True,
    derivative_on_measurement=True,
    derivative_filter_tau=0.05,
)

plant = MassSpringDamper(
    mass=1.0,
    damping=10.0,
    spring=20.0,
)

results = simulate(
    controller,
    plant,
    duration=10.0,
    setpoint=1.0,
    plant_dt=0.001,
)

print("Final position:", results["position"][-1])
```

The simulation returns NumPy arrays containing time,
setpoint, position, velocity, sensor measurement,
tracking error, applied force, and individual PID terms.

## Reproducibility

All experiments use fixed plant parameters, controller
configurations, and simulation intervals defined within
their respective Python files.

The sensor noise experiment uses a fixed random seed.

Every figure is saved as a PNG image at 300 DPI.

To reproduce all figures, install the dependencies
and execute:

```bash
python run_experiments.py
```

Numerical results are printed directly by the
experiments, allowing the reported performance metrics
to be compared against the generated graphs.

## Limitations

- The primary plant assumes linear spring stiffness
  and viscous damping.
- Actuator saturation is enabled only in experiments
  where force limits are being investigated.
- Sensor noise is modeled as bounded, uniformly
  distributed random noise.
- Steady-state error is estimated using a finite
  simulation duration.
- Continuous-time stability calculations may differ
  from the actual sampled-controller stability limits.
- The selected gains are specific to the simulated
  plant parameters and operating conditions.
- The results represent numerical simulations rather
  than physical hardware measurements.

## Conclusion

Proportional control improves the response to present
error but cannot eliminate steady-state error in the
given mass-spring-damper system.

Integral control removes persistent error, while
derivative control can improve damping and reduce
oscillations.

Practical PID implementations must also consider
actuator limits, integral windup, sensor noise,
derivative kick, and controller sampling intervals.

Ziegler-Nichols tuning provides an initial controller
configuration that can be refined according to
application-specific performance requirements.

## AI Assistance

AI tools were used to assist with implementation planning,
Python code development, mathematical explanations, and
documentation.

The controller logic and simulation methods are presented
explicitly to support independent understanding, execution,
modification, and verification.

## References

- Åström, K. J., and Murray, R. M.
  *Feedback Systems: An Introduction for Scientists
  and Engineers.*
  https://fbsbook.org/

- Ogata, K.
  *Modern Control Engineering.*

- NumPy Documentation:
  https://numpy.org/doc/

- Matplotlib Documentation:
  https://matplotlib.org/stable/
