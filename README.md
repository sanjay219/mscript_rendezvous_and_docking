# Spacecraft Rendezvous and R-Bar Docking Simulation

## Overview

This project implements a **modular MATLAB framework for spacecraft
rendezvous and docking** using the R-bar approach.

The simulation models the motion of a **chaser spacecraft approaching a
passive target spacecraft** in Low Earth Orbit (LEO).\
The mission is divided into two main phases:

1.  **Rendezvous Phase** The chaser performs optimized impulsive burns
    to reach a position **1 km below the target spacecraft**.

2.  **Docking Phase (R-bar approach)** The chaser performs a controlled
    approach toward the target using a **feedback controller** while
    maintaining safe approach speeds.

The simulation also includes **3D visualization using MATLAB
`satelliteScenario`**.

------------------------------------------------------------------------

# Simulation Architecture

    Main Script (run this file only : main_rendezvous_docking.m)
    │
    ├── Mission Setup
    │
    ├── Rendezvous Optimization
    │   ├── fsolve initial guess
    │   └── fmincon optimization
    │
    ├── Relative Motion Dynamics
    │   └── Clohessy–Wiltshire (CW) equations
    │
    ├── Docking Guidance
    │   └── R-bar approach logic
    │
    ├── Controllers
    │   ├── Translational LQR controller
    │   └── Quaternion PD attitude controller
    │
    ├── Rigid Body Attitude Dynamics
    │
    ├── Coordinate Transformations
    │
    └── Visualization (satelliteScenario)

------------------------------------------------------------------------


## Environment and Physical Constants

| Parameter | Symbol | Value | Unit | Description |
|---|---:|---:|---|---|
| Earth gravitational parameter | $\mu$ | $3.986004418 \times 10^{14}$ | m³/s² | Standard gravitational parameter of Earth |
| Earth radius | $R_e$ | 6,378,137 | m | Mean Earth radius |

---

## Target Spacecraft Orbit

| Parameter | Symbol | Value | Unit | Description |
|---|---:|---:|---|---|
| Orbit altitude | $h$ | 500,000 | m | Target orbit altitude |
| Orbit radius | $a_T$ | 6,878,137 | m | Circular orbit radius |
| Inclination | $i$ | 30 | deg | Orbital inclination |
| Right Ascension of Ascending Node | $\Omega$ | 40 | deg | Orbital plane orientation |
| Argument of periapsis | $\omega$ | 0 | deg | Orbit orientation |
| True anomaly | $\nu_0$ | 20 | deg | Initial orbital position |
| Mean motion | $n$ | $\approx 0.0011$ | rad/s | Orbital angular velocity |
| Orbital period | $T$ | $\approx 5660$ | s | Approximately 94 minutes |

---

## Relative Motion Initial Conditions

The simulation uses the **Hill frame** (also called the **LVLH frame**).

### Hill Frame Axes

| Axis | Description |
|---|---|
| $x$ | Radial direction (R-bar) |
| $y$ | Along-track direction (V-bar) |
| $z$ | Cross-track direction |

### Initial Chaser State

| Parameter | Value | Unit | Description |
|---|---:|---|---|
| Radial offset | -5000 | m | Chaser starts 5 km below target |
| Along-track offset | 100 | m | Small along-track displacement |
| Cross-track offset | 50 | m | Small cross-track displacement |
| Relative velocity | $[0,\,0,\,0]$ | m/s | Initial relative velocity |

### Desired Rendezvous State

| Parameter | Value | Unit | Description |
|---|---:|---|---|
| Radial position | -1000 | m | 1 km below target |
| Relative velocity | $[0,\,0,\,0]$ | m/s | Zero relative velocity |

---

## Spacecraft Physical Properties

| Parameter | Symbol | Value | Unit | Description |
|---|---:|---:|---|---|
| Moment of inertia (x) | $I_x$ | 120 | kg·m² | Spacecraft inertia |
| Moment of inertia (y) | $I_y$ | 100 | kg·m² | Spacecraft inertia |
| Moment of inertia (z) | $I_z$ | 80 | kg·m² | Spacecraft inertia |
| Initial quaternion | $q_0$ | $[1,\,0,\,0,\,0]$ | — | Identity quaternion |
| Initial angular velocity | $\omega_0$ | $[0,\,0,\,0]$ | rad/s | No initial rotation |

> In the current baseline model, **mass is not explicitly used in translational propagation** because the CW docking controller is implemented in **acceleration form** rather than force form.  
> If required, mass can be introduced later to convert thrust force into acceleration and to model propellant depletion.

---

## Rendezvous Optimization

The rendezvous maneuver is solved using **nonlinear constrained optimization**.

### Optimization Variables

| Variable | Description |
|---|---|
| First burn time | Time of first impulsive maneuver |
| Second burn time | Time after first burn |
| $\Delta v_1$ | First burn velocity vector |
| $\Delta v_2$ | Second burn velocity vector |

### Objective Function

The optimization minimizes total impulsive maneuver cost:

$$
J = \lVert \Delta v_1 \rVert^2 + \lVert \Delta v_2 \rVert^2
$$

### Optimization Process

1. Generate an initial feasible guess using **`fsolve`**
2. Refine the solution using **`fmincon`**

### Bounds Used in the Example

| Parameter | Value | Unit |
|---|---:|---|
| Minimum first burn time | 200 | s |
| Maximum first burn time | 86,400 | s |
| Burn component bounds | $\pm 5$ | m/s |

---

## Relative Motion Model

Relative motion between chaser and target is modeled using the **Clohessy–Wiltshire (CW) equations**.

State vector:

$$
x =
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z}
\end{bmatrix}^T
$$

Continuous-time state-space form:

$$
\dot{x} = A x + B u
$$

with

$$
A =
\begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
3n^2 & 0 & 0 & 0 & 2n & 0 \\
0 & 0 & 0 & -2n & 0 & 0 \\
0 & 0 & -n^2 & 0 & 0 & 0
\end{bmatrix},
\qquad
B =
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

where:

- $x, y, z$ are relative position coordinates in the Hill frame
- $\dot{x}, \dot{y}, \dot{z}$ are relative velocity components
- $u = [u_x, u_y, u_z]^T$ is commanded control acceleration

---

## Docking Controller

### Translational Controller

An **LQR (Linear Quadratic Regulator)** controller is used for relative position tracking during docking.

Controller law:

$$
u = -K(x - x_d)
$$

where:

- $x$ is the current relative state
- $x_d$ is the desired relative state
- $K$ is the optimal state feedback gain matrix

### Weighting Matrices

| Matrix | Value |
|---|---|
| $Q$ | `diag(5e-4, 5e-3, 5e-3, 1, 8, 8)` |
| $R$ | `diag(1, 1, 1)` |

---

## Attitude Model and Controller

Spacecraft attitude is modeled using **quaternion kinematics** and **rigid-body rotational dynamics**.

### Quaternion Kinematics

$$
\dot{q} = \frac{1}{2}\Omega(\omega) q
$$

### Rigid Body Dynamics

$$
I \dot{\omega} + \omega \times (I\omega) = \tau
$$

### Quaternion PD Control

$$
\tau = -K_q\,q_e^{vec} - K_\omega\,\omega_e
$$

where:

- $q_e^{vec}$ is the vector part of the quaternion error
- $\omega_e$ is the angular velocity error
- $\tau$ is the control torque

---

## Thruster and Docking Limits

| Parameter | Value | Unit | Description |
|---|---:|---|---|
| Maximum translational acceleration | 0.02 | m/s² | Saturation limit on docking acceleration |
| Initial closing velocity | 0.3 | m/s | Nominal approach speed |
| Final closing velocity | 0.01 | m/s | Reduced speed near docking |
| Docking safety radius | 3 | m | Keep-out / contact threshold used for visual-safe docking |

---

## Numerical Integration

| Parameter | Value |
|---|---|
| Integrator | RK4 |
| Time step | 2 s |

---

## Visualization

The mission is visualized using MATLAB **`satelliteScenario`**.

Visualization features include:

- 3D spacecraft models
- Colored orbit trajectories
- Docking port visualization
- Real-time mission playback

---

## Simulation Phases

| Phase | Description |
|---|---|
| Initial separation | Approximately 5 km |
| Rendezvous target | 1 km below target |
| Controlled approach | 1 km → 20 m |
| Final docking approach | 20 m → docking safety radius |
| Docking completion | Simulation stops when docking threshold is reached |

---

## Assumptions

The current framework is a **baseline research model** and uses the following assumptions:

- Circular reference orbit
- Linearized CW relative dynamics
- Impulsive rendezvous burns
- Acceleration-based thruster model
- Simplified collision envelope / keep-out radius
- No propellant depletion in docking control
- No J2 or higher-order orbital perturbations

---

This simulation demonstrates:

1.  Optimized rendezvous using impulsive burns
2.  Controlled R-bar docking approach
3.  Feedback control using LQR and quaternion PD
4.  Visualization using MATLAB satelliteScenario

# File Structure

1) Main mission script
main_rendezvous_docking.m

This is the top-level script that runs the complete mission.
It defines orbital parameters, initial relative conditions, optimization setup, docking controller settings, attitude parameters, plotting, and satelliteScenario visualization.
In short, this file connects everything together.

2) Rendezvous optimization files
solve_initial_guess_fsolve.m

This file uses fsolve to generate an initial feasible solution for the rendezvous maneuver.
Its job is to satisfy the final rendezvous conditions approximately before optimization.
This helps the next optimization stage converge better.

solve_optimal_burns_fmincon.m

This file uses fmincon to compute the optimized two-burn rendezvous solution.
It minimizes total delta-v while satisfying terminal conditions and variable bounds.
This is the main fuel-minimization step.

objective_dv.m

This file defines the optimization cost function.
It computes the cost as the sum of the squared magnitudes of the two burn vectors.
So this file is the mathematical expression of “minimize total delta-v.”

constraint_rendezvous.m

This file defines the nonlinear constraints for the rendezvous problem.
It mainly enforces the final relative state requirement after the two burns.
It is the place where future safety or corridor constraints can be added.

rendezvous_constraint_eq.m

This file computes the equality constraints only for the rendezvous stage.
It propagates the motion through the first burn, coast phase, and second burn, then compares the final state to the desired rendezvous state.
This file is especially useful for the fsolve stage.

3) Relative-motion propagation files
propagate_two_impulse_cw.m

This file propagates the entire rendezvous trajectory under the two-impulse CW model.
It handles three parts: before burn 1, between burn 1 and burn 2, and after burn 2.
This gives the complete relative trajectory history for the rendezvous phase.

cw_propagate.m

This file propagates the CW relative state over a given time interval using the CW state transition.
It is a compact helper for linear relative motion propagation.
This is one of the core translational dynamics utilities.

cwA.m

This file returns the CW system matrix A.
It encodes the linearized relative dynamics around the circular target orbit.
It is the mathematical core of the Clohessy–Wiltshire model.

cw_state_matrices.m

This file returns both the A and B matrices of the CW state-space model.
These are used when building the translational controller, especially the LQR design.
This file connects the physics model to the controller design.

cw_dynamics.m

This file computes the time derivative of the CW state.
It includes the relative motion equations plus control acceleration input.
This is the continuous-time plant model used during docking.

rk4_cw_step.m

This file performs one Runge–Kutta 4th-order integration step for the CW dynamics.
It is used to numerically propagate the docking phase under feedback control.
This file makes the time stepping more stable and accurate than simple Euler integration.

4) Docking guidance and translational control files
translational_lqr_controller.m

This file implements the LQR feedback control law for translational motion.
It computes control acceleration using the relative state error between current and desired state.
This is the main docking translation controller.

guidance_docking_phase.m

This file provides the desired position and velocity commands for each docking phase.
It switches between initial approach, transposition, and final approach waypoints.
In short, this file decides where the chaser should go next.

5) Attitude control and rigid-body dynamics files
attitude_pd_controller.m

This file implements the quaternion PD attitude controller.
It computes control torque from quaternion error and angular velocity error.
This is the main rotational controller for target/chaser pointing.

rigid_body_dynamics.m

This file models the spacecraft rotational rigid-body dynamics.
It computes quaternion kinematics and angular acceleration under applied torque and inertia.
This is the rotational equivalent of the translational plant model.

quat_kinematics.m

This file computes quaternion time derivatives from angular velocity.
It is the standard quaternion kinematics equation used in spacecraft attitude propagation.
This file keeps orientation updated without Euler-angle singularities.

rk4_attitude_step.m

This file performs one RK4 integration step for spacecraft attitude and angular rate.
It numerically propagates quaternion and body rates under the applied control torque.
This is the rotational integrator used in the docking simulation.

desired_chaser_attitude_from_relative_state.m

This file generates the desired chaser attitude based on the relative position to the target.
Its purpose is to point the chaser appropriately during docking, especially toward the line of sight.
This makes the chaser body frame rotate to maintain docking alignment.

6) Quaternion utility files
quat_conj.m

This file computes the quaternion conjugate.
It is used when forming attitude errors and inverse rotations.
This is a small but essential quaternion utility.

quat_multiply.m

This file performs quaternion multiplication.
It combines rotations and is also used in attitude error calculations.
This is another core quaternion math helper.

rotm_to_quat_scalar_first.m

This file converts a rotation matrix into a scalar-first quaternion.
It is used when desired attitude is first built in matrix form and then converted to quaternion form.
This bridges geometric frame construction and controller input format.

7) Orbital geometry / coordinate transformation files
build_eci_histories.m

This file converts the relative Hill-frame motion into ECI position histories for both target and chaser.
It builds the trajectories needed for satelliteScenario visualization.
This is the file that connects the relative-motion simulation to the inertial-space animation.

circularOrbitPropagate.m

This file propagates the target spacecraft along its circular reference orbit.
It advances true anomaly with time and returns inertial position and velocity.
This is how the target orbit is generated in the simulation.

oe2rv_circular.m

This file converts circular orbital elements into inertial position and velocity vectors.
It is a standard orbital mechanics utility.
This file supports the target orbit initialization and propagation.
