# Control Systems Summary — TTHopper 6-DOF Rocket Simulation

> **Purpose.**  This document is a self-contained introduction to every
> control-theory concept used in the TTHopper simulation.  It is written
> for readers who are new to control engineering and aims to build
> understanding from the ground up — starting with what "controlling"
> means in general, introducing the physics that the controllers must
> tame, and then walking through each of the four flight-phase controllers
> with their full mathematical derivations.

---

## Table of Contents

1. [What Is a Control System?](#1-what-is-a-control-system)
2. [Project Architecture — Which File Does What](#2-project-architecture--which-file-does-what)
3. [The Plant — 6-DOF Rigid-Body Dynamics](#3-the-plant--6-dof-rigid-body-dynamics)
   - 3.1 [State Vector](#31-state-vector)
   - 3.2 [Quaternion Orientation](#32-quaternion-orientation)
   - 3.3 [Translational Dynamics (Newton)](#33-translational-dynamics-newton)
   - 3.4 [Rotational Dynamics (Euler)](#34-rotational-dynamics-euler)
   - 3.5 [Propulsion Model](#35-propulsion-model)
   - 3.6 [Aerodynamic Model](#36-aerodynamic-model)
   - 3.7 [Numerical Integration — RK4](#37-numerical-integration--rk4)
4. [Mission Phases Overview](#4-mission-phases-overview)
5. [Phase 1a — Cascaded PD Controller (Powered Ascent)](#5-phase-1a--cascaded-pd-controller-powered-ascent)
   - 5.1 [What Is PD / PID Control?](#51-what-is-pd--pid-control)
   - 5.2 [Why "Cascaded"?](#52-why-cascaded)
   - 5.3 [Outer Loop — Position → Force](#53-outer-loop--position--force)
   - 5.4 [Middle Loop — Force → Attitude + Throttle](#54-middle-loop--force--attitude--throttle)
   - 5.5 [Inner Loop — Attitude → Gimbal](#55-inner-loop--attitude--gimbal)
   - 5.6 [Gain Derivation from Plant Dynamics](#56-gain-derivation-from-plant-dynamics)
   - 5.7 [Roll Control via Ailerons](#57-roll-control-via-ailerons)
6. [Phase 1b — Coast (Prograde Hold)](#6-phase-1b--coast-prograde-hold)
7. [Phase 1c — MPC Flip Manoeuvre](#7-phase-1c--mpc-flip-manoeuvre)
   - 7.1 [What Is Model Predictive Control?](#71-what-is-model-predictive-control)
   - 7.2 [Prediction Model](#72-prediction-model)
   - 7.3 [Cost Function](#73-cost-function)
   - 7.4 [Optimisation and Warm-Starting](#74-optimisation-and-warm-starting)
8. [Phase 2 — Sliding-Mode Controller (Ballistic Descent)](#8-phase-2--sliding-mode-controller-ballistic-descent)
   - 8.1 [What Is Sliding-Mode Control?](#81-what-is-sliding-mode-control)
   - 8.2 [The Sliding Surface](#82-the-sliding-surface)
   - 8.3 [Control Law with Boundary Layer](#83-control-law-with-boundary-layer)
   - 8.4 [Dynamic-Pressure Scaling](#84-dynamic-pressure-scaling)
9. [Phase 3 — LQR Landing Controller](#9-phase-3--lqr-landing-controller)
   - 9.1 [Suicide-Burn Throttle Logic](#91-suicide-burn-throttle-logic)
   - 9.2 [Retrograde-to-Vertical Attitude Blending](#92-retrograde-to-vertical-attitude-blending)
   - 9.3 [What Is LQR?](#93-what-is-lqr)
   - 9.4 [Linearised Plant Model (A and B Matrices)](#94-linearised-plant-model-a-and-b-matrices)
   - 9.5 [The Riccati Equation](#95-the-riccati-equation)
   - 9.6 [Aileron Pitch / Yaw Assist](#96-aileron-pitch--yaw-assist)
   - 9.7 [PD Fallback](#97-pd-fallback)
10. [Phase Transitions — When Does the Phase Change?](#10-phase-transitions--when-does-the-phase-change)
11. [Reference Trajectory — Hermite-Parameterised Quarter-Ellipse](#11-reference-trajectory--hermite-parameterised-quarter-ellipse)
12. [Control-Vector Summary](#12-control-vector-summary)
13. [Parameter Table](#13-parameter-table)
14. [Glossary](#14-glossary)

---

## 1. What Is a Control System?

A **control system** is a mechanism that drives a **plant** (the thing
being controlled) toward a desired behaviour by continuously measuring
its state, comparing the measurement to a reference, and computing a
corrective **command**.

```
  reference ─→ [Controller] ─→ command ─→ [Plant] ─→ output
                    ↑                                  │
                    └───────── measurement ─────────────┘
```

This closed loop is called **feedback control**.  The difference between
what we *want* and what we *measure* is called the **error**:

$$e(t) = r(t) - y(t)$$

where $r$ is the reference, $y$ the output, and $e$ the error.  Every
controller in this project is a feedback controller: it reads the
rocket's current state (position, velocity, orientation, angular rate),
compares it to a desired target, and outputs engine/fin commands.

The **plant** in our case is the *6-DOF rigid-body rocket* — the
physical system that responds to thrust, gravity, and aerodynamic
forces.  The controllers have no authority over gravity or the wind;
they can only set:

| Actuator | Controlled variable | Range |
|---|---|---|
| Throttle | Engine thrust magnitude | 0 – 100 % of 200 kN |
| Gimbal Y / Z | Nozzle deflection angles | ±7.5 ° |
| Ailerons (grid fins) | Direct body-frame torques | ±5 000 N·m per axis |

---

## 2. Project Architecture — Which File Does What

```
src/
├── config.py                  All numerical parameters in one place
├── math_utils.py              Quaternion algebra, rotation helpers
├── physics/
│   ├── environment.py         Atmosphere density ρ(h), gravity
│   ├── propulsion.py          Thrust vector & moment from gimbal angles
│   ├── aerodynamics.py        Lift, drag, aero moment about CG
│   └── dynamics.py            6-DOF state derivative  ẋ = f(x,u)
├── models/
│   ├── rocket.py              (reserved for mass properties)
│   └── trajectory.py          Reference quarter-ellipse for ascent
├── controllers/
│   ├── base.py                Abstract interface every controller shares
│   ├── ascent_pid.py          Cascaded PD  — Phases 1a & 1b
│   ├── flip_mpc.py            Model Predictive Control — Phase 1c
│   ├── ballistic_smc.py       Sliding-Mode Control — Phase 2
│   └── landing_lqr.py         LQR + suicide-burn — Phase 3
├── simulation/
│   ├── simulator.py           RK4 integration loop, phase dispatch
│   ├── phase_manager.py       Detects phase transitions, saves snapshots
│   └── data_logger.py         Stores time histories for plotting
└── visualization/
    └── plotter.py             3-D trajectory & telemetry animation
```

**Data flow in one time-step:**

1. `simulator.py` asks `phase_manager.py` what phase we are in.
2. `simulator.py` calls the appropriate controller's `compute()` method.
3. The controller reads the 13-element **state** and returns a 6-element **control**.
4. `dynamics.py` uses the control and the state to compute the
   **state derivative** $\dot{x} = f(x, u)$.
5. The RK4 integrator in `simulator.py` advances the state by one time-step $\Delta t$.
6. The new state is passed back to step 1 for the next cycle.

---

## 3. The Plant — 6-DOF Rigid-Body Dynamics

### 3.1 State Vector

The rocket's full state is a single 13-element vector:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ z \\ v_x \\ v_y \\ v_z \\ q_w \\ q_x \\ q_y \\ q_z \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

| Indices | Symbol | Meaning |
|---|---|---|
| 0 – 2 | $x, y, z$ | Position in world frame (ENU: East, North, Up) |
| 3 – 5 | $v_x, v_y, v_z$ | Velocity in world frame |
| 6 – 9 | $q_w, q_x, q_y, q_z$ | Orientation quaternion (body → world) |
| 10 – 12 | $\omega_x, \omega_y, \omega_z$ | Angular velocity in body frame |

The world frame has **Z pointing up** (altitude).  The body frame has
**$z_B$ along the rocket's longitudinal axis** (engine → nose).  At
launch, both frames coincide and $q = [1,0,0,0]$.

### 3.2 Quaternion Orientation

**Why quaternions instead of Euler angles?**  Euler angles suffer from
*gimbal lock* — a singularity at $\pm 90°$ pitch where yaw and roll
become indistinguishable.  During the 180° flip, the rocket passes
through this singularity, which would cause the simulation to crash.
Quaternions are a four-component representation of orientation that has
no singularities for any rotation.

A quaternion is written as:

$$q = w + x\,\mathbf{i} + y\,\mathbf{j} + z\,\mathbf{k}$$

We use the **scalar-first Hamilton convention**: $q = [w, x, y, z]$.
A *unit* quaternion ($\|q\| = 1$) encodes a rotation.  The identity
rotation (no rotation) is $q = [1, 0, 0, 0]$.

**Key operations** (implemented in `math_utils.py`):

| Operation | Formula | Code function |
|---|---|---|
| Hamilton product | $q_1 \otimes q_2$ (rotation composition) | `quat_multiply` |
| Conjugate / inverse | $q^* = [w, -x, -y, -z]$ | `quat_conjugate` |
| Rotate a vector | $v' = R(q) \cdot v$ via DCM | `quat_rotate` |
| Attitude error | $q_e = q_\mathrm{des}^* \otimes q_\mathrm{act}$ | `quat_error_vec` |
| Quaternion derivative | $\dot{q} = \tfrac{1}{2} q \otimes [0, \boldsymbol{\omega}]$ | `omega_to_quat_deriv` |

The **Direction Cosine Matrix** (DCM), $R$, converts body-frame vectors
to world-frame vectors: $\mathbf{v}_\text{world} = R \, \mathbf{v}_\text{body}$.
It is a 3 × 3 orthogonal matrix computed from the four quaternion components
(see `quat_to_dcm` in `math_utils.py`).

**Attitude error** is the key quantity every controller needs.  Given the
desired quaternion $q_d$ and the actual quaternion $q_a$, the error
quaternion is:

$$q_e = q_d^{*} \otimes q_a$$

For small errors, we extract a 3-element error vector (body frame):

$$\mathbf{e}_\text{att} = 2 \, [q_{e,x},\; q_{e,y},\; q_{e,z}]$$

This vector points along the axis the rocket must rotate about, and its
magnitude is approximately the angle (in radians) of the misalignment.
When $\mathbf{e}_\text{att} = \mathbf{0}$, the orientations match.

### 3.3 Translational Dynamics (Newton)

Newton's second law in the world frame:

$$m \, \dot{\mathbf{v}} = \mathbf{F}_\text{gravity} + \mathbf{F}_\text{thrust} + \mathbf{F}_\text{aero}$$

- **Gravity:** $\mathbf{F}_g = [0,\;0,\;-m g]$ with $g = 9.81\;\text{m/s}^2$.
- **Thrust:** The engine produces a force in the body frame (see §3.5).
  It is rotated to the world frame by the DCM:
  $\mathbf{F}_\text{thrust,world} = R \, \mathbf{F}_\text{thrust,body}$.
- **Aerodynamics:** Lift and drag forces in the world frame (see §3.6).

The position simply integrates velocity: $\dot{\mathbf{r}} = \mathbf{v}$.

### 3.4 Rotational Dynamics (Euler)

Euler's rotation equation in the body frame:

$$I \, \dot{\boldsymbol{\omega}} = \mathbf{M} - \boldsymbol{\omega} \times (I\,\boldsymbol{\omega})$$

where:

- $I = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the **inertia tensor**
  (diagonal because the rocket is axially symmetric).
- $\mathbf{M}$ is the total moment about the centre of gravity
  (thrust moment + aero moment + aileron torque), all in the body frame.
- $\boldsymbol{\omega} \times (I\boldsymbol{\omega})$ is the **gyroscopic
  coupling** term — it encodes the fact that angular momentum precesses
  when the rocket spins on more than one axis simultaneously.

The quaternion derivative (§3.2) propagates the orientation:

$$\dot{q} = \frac{1}{2}\, q \otimes [0, \boldsymbol{\omega}]$$

Both equations are solved simultaneously inside `dynamics.py`.

### 3.5 Propulsion Model

The engine sits at the bottom of the rocket.
Its nozzle can be *gimballed* (tilted) by two angles:

| Angle | Name | Plane of Action |
|---|---|---|
| $\delta_y$ (`gimbal_y`) | Nozzle deflection about body-Y | X-Z plane |
| $\delta_z$ (`gimbal_z`) | Nozzle deflection about body-X | Y-Z plane |

The thrust force in the **body frame** is:

$$\mathbf{F}_\text{body}
= T \begin{bmatrix} \sin\delta_y \\ -\sin\delta_z \\ \cos\delta_y \,\cos\delta_z \end{bmatrix}$$

where $T = \text{throttle} \times F_\max$.  When both gimbal angles are
zero, thrust points purely along $+z_B$ (toward the nose).

The **moment** about the CG is the cross product of the engine position
vector with the thrust force:

$$\mathbf{r}_\text{engine} = \begin{bmatrix} 0 \\ 0 \\ -L \end{bmatrix}, \quad
\mathbf{M}_\text{body} = \mathbf{r}_\text{engine} \times \mathbf{F}_\text{body}$$

where $L = 2.913\;\text{m}$ is the distance from the engine to the CG.
Expanding the cross product:

$$\mathbf{M} = \begin{bmatrix}
(-L)\cdot(-T\sin\delta_z) - 0 \\
0 - (-L)\cdot(T\sin\delta_y) \\
0
\end{bmatrix}
= \begin{bmatrix}
-L\,T\,\sin\delta_z \\
-L\,T\,\sin\delta_y \\
0
\end{bmatrix}$$

> **Wait — the cross product.**
> For $\mathbf{r} = [0, 0, -L]$ and $\mathbf{F} = [F_x, F_y, F_z]$:
>
> $$\mathbf{M} = \mathbf{r} \times \mathbf{F}
> = \begin{bmatrix}
> r_y F_z - r_z F_y \\
> r_z F_x - r_x F_z \\
> r_x F_y - r_y F_x
> \end{bmatrix}
> = \begin{bmatrix}
> 0 \cdot F_z - (-L) \cdot F_y \\
> (-L) \cdot F_x - 0 \cdot F_z \\
> 0
> \end{bmatrix}
> = \begin{bmatrix}
> L\,F_y \\
> -L\,F_x \\
> 0
> \end{bmatrix}$$
>
> Substituting $F_x = T\sin\delta_y$ and $F_y = -T\sin\delta_z$:
>
> $$M_x = L \cdot (-T\sin\delta_z) = -L\,T\,\sin\delta_z$$
> $$M_y = -L \cdot T\sin\delta_y = -L\,T\,\sin\delta_y$$

This sign convention is critical:

- **Both** $M_x$ and $M_y$ are **negative** for positive gimbal angles.
- A positive `gimbal_y` creates a **negative** pitch moment $M_y$.
- A positive `gimbal_z` creates a **negative** roll moment $M_x$.

The controllers must account for this **sign inversion** (see §5.5).

### 3.6 Aerodynamic Model

The atmosphere follows an exponential density profile:

$$\rho(h) = \rho_0 \, e^{-h/H}$$

with $\rho_0 = 1.225\;\text{kg/m}^3$ (sea level) and scale height
$H = 8500\;\text{m}$.  **Dynamic pressure** is:

$$q_\infty = \frac{1}{2} \rho \, v^2$$

**Drag** always opposes velocity:

$$\mathbf{F}_\text{drag} = -q_\infty \, S_\text{ref} \, C_D \, \hat{\mathbf{v}}$$

where $C_D = C_{D_0} + C_{D_\alpha}\,\alpha^2$ depends on the **angle of
attack** $\alpha$ (angle between velocity and body axis).

**Lift** acts perpendicular to velocity, in the plane containing
the body axis:

$$\mathbf{F}_\text{lift} = q_\infty \, S_\text{ref} \, C_{L_\alpha} \, \alpha \; \hat{\mathbf{l}}$$

The **centre of pressure** (CP) — the point where the aero force effectively
acts — shifts depending on phase:

| Phase | CP offset from CG | Stability character |
|---|---|---|
| Ascent (1a, 1b, 1c) | +0.5 m (toward nose) | **Unstable** — CP ahead of CG, aero moment amplifies tilts |
| Descent (2, 3) | −1.5 m (behind CG → toward tail) | **Stable** — grid fins move CP behind CG, aero restores orientation |

The aero **moment** about the CG is:

$$\mathbf{M}_\text{aero} = \mathbf{r}_\text{CP} \times \mathbf{F}_\text{aero,body}$$

where $\mathbf{r}_\text{CP}$ is the CP offset vector in body coordinates.

### 3.7 Numerical Integration — RK4

The state derivative $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ is integrated
using the **4th-order Runge-Kutta method** (RK4) inside `simulator.py`:

$$k_1 = f(x_n, u)$$
$$k_2 = f(x_n + \tfrac{\Delta t}{2} k_1, u)$$
$$k_3 = f(x_n + \tfrac{\Delta t}{2} k_2, u)$$
$$k_4 = f(x_n + \Delta t \, k_3, u)$$
$$x_{n+1} = x_n + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

with $\Delta t = 0.02\;\text{s}$ (50 Hz).  After each step, the
quaternion part is **re-normalised** to stop drift:

$$q \leftarrow \frac{q}{\|q\|}$$

RK4 is chosen for its balance of accuracy and simplicity.  It is
fourth-order, meaning the error per step scales with $(\Delta t)^5$.

---

## 4. Mission Phases Overview

The mission is divided into six sequential phases.  Each transition is
detected automatically by `phase_manager.py`.

```
 ┌──────────┐     ┌──────────┐     ┌──────────┐
 │ Phase 1a │────→│ Phase 1b │────→│ Phase 1c │
 │ Powered  │     │ Coast to │     │ Boostback│
 │ Ascent   │     │ Apogee   │     │ Flip     │
 └──────────┘     └──────────┘     └──────────┘
                                        │
                ┌──────────┐     ┌──────┴─────┐
                │ Phase 3  │←────│ Phase 2    │
                │ Powered  │     │ Ballistic  │
                │ Landing  │     │ Descent    │
                └────┬─────┘     └────────────┘
                     │
                ┌────┴─────┐
                │ LANDED   │
                └──────────┘
```

| Phase | Controller | Actuators Used | Goal |
|---|---|---|---|
| **1a** | Cascaded PD | Engine (throttle + gimbal) + ailerons | Follow quarter-ellipse trajectory from pad to MECO |
| **1b** | Cascaded PD (coast mode) | Ailerons only | Hold prograde orientation, engine off |
| **1c** | MPC | Engine (25 % throttle + gimbal) + ailerons | 180° flip to retrograde |
| **2** | Sliding-Mode | Ailerons only | Hold retrograde attitude during free-fall |
| **3** | LQR + PD | Engine (throttle + gimbal) + ailerons | Suicide-burn to soft landing |
| **Landed** | None | — | Terminal state |

---

## 5. Phase 1a — Cascaded PD Controller (Powered Ascent)

**File:** `src/controllers/ascent_pid.py`

### 5.1 What Is PD / PID Control?

The simplest feedback controller is **proportional** (P): the command is
proportional to the error:

$$u = K_P \, e$$

If $K_P$ is large, the response is fast but may overshoot.

Adding a **derivative** term dampens oscillations:

$$u = K_P \, e + K_D \, \dot{e}$$

This is **PD control**.  The derivative term resists rapid changes —
it acts like a viscous damper.

A full **PID** controller also includes an **integral** term
($K_I \int e\,dt$) that eliminates steady-state error, but our
simulation does not need it because there is no persistent bias
(e.g., wind) for the rocket.

### 5.2 Why "Cascaded"?

The rocket cannot directly set its position — it can only tilt and
throttle.  A single PD loop from position error to gimbal angle would
be very difficult to tune because the mapping is indirect and nonlinear.

A **cascade** breaks the problem into nested loops, each one simpler
than the whole:

```
 ┌───────────────────────────────────────────────────────────┐
 │  OUTER LOOP          MIDDLE LOOP         INNER LOOP      │
 │                                                           │
 │  pos_error ──→ desired  force ──→ desired   att_err ──→  │
 │  vel_error     velocity  mag     attitude   ω            │
 │               direction         (quat)     gimbal        │
 │                throttle                    ailerons      │
 └───────────────────────────────────────────────────────────┘
```

Each inner loop runs faster and controls a simpler plant.  The outer
loop "thinks" at the position level; the inner loop "thinks" at the
angular rate level.

### 5.3 Outer Loop — Position → Force

The controller knows where the rocket *should* be (from the reference
trajectory) and where it *is*.

$$\mathbf{e}_\text{pos} = \mathbf{r}_\text{ref}(t) - \mathbf{r}_\text{actual}$$

A desired velocity is formed by adding the reference velocity and a
correction proportional to the position error:

$$\mathbf{v}_\text{des} = \mathbf{v}_\text{ref}(t) + K_{P,\text{pos}} \, \mathbf{e}_\text{pos}$$

The velocity error drives a desired acceleration:

$$\mathbf{a}_\text{des} = K_{D,\text{vel}} \, (\mathbf{v}_\text{des} - \mathbf{v}_\text{actual})$$

Finally, **gravity compensation** is added to get the desired force:

$$\mathbf{F}_\text{des} = m \left( \mathbf{a}_\text{des} + \begin{bmatrix}0\\0\\g\end{bmatrix} \right)$$

The $+g$ term ensures the rocket exerts enough thrust just to hover
before any correction is applied.

**Gains used:** $K_{P,\text{pos}} = [0.3,\;0.3,\;0.5]$, $K_{D,\text{vel}} = [1.5,\;1.5,\;2.5]$.

### 5.4 Middle Loop — Force → Attitude + Throttle

The desired force vector has a magnitude and a direction.

**Throttle:** The engine must deliver the required magnitude.

$$\text{throttle} = \text{clip}\!\left(\frac{\|\mathbf{F}_\text{des}\|}{F_\max},\;0.05,\;1.0\right)$$

**Desired orientation:** The rocket's $z_B$ axis must be aligned with
$\hat{\mathbf{F}}_\text{des}$ (the normalised desired-force direction).
The function `align_body_z_to()` computes the quaternion that
rotates world-Z to this direction:

$$q_\text{des} = \text{align\_body\_z\_to}(\hat{\mathbf{F}}_\text{des})$$

### 5.5 Inner Loop — Attitude → Gimbal

The attitude error is a 3-element body-frame vector (§3.2):

$$\mathbf{e}_\text{att} = \text{quat\_error\_vec}(q_\text{des}, q_\text{actual})$$

The PD law maps this to gimbal commands:

$$\delta_y = \text{clip}\!\left(K_P \, e_y + K_D \, \omega_y,\;\pm\delta_\max\right)$$
$$\delta_z = \text{clip}\!\left(K_P \, e_x + K_D \, \omega_x,\;\pm\delta_\max\right)$$

**Notice the axis mapping:** `gimbal_y` corrects pitch error ($e_y$)
and `gimbal_z` corrects roll/yaw error ($e_x$).  This is because
`gimbal_y` rotates the nozzle about the body-Y axis, which produces
a moment that rotates the rocket in pitch (about Y).

**Notice the sign:** The formula uses $+K_P \, e$ instead of $-K_P \, e$.
This accounts for the **geometric sign inversion** (§3.5): a positive
gimbal angle produces a *negative* moment.  Since we want a positive
error to drive a negative moment (corrective), we must command a
positive gimbal angle — hence $+K_P$.

### 5.6 Gain Derivation from Plant Dynamics

The gains are not arbitrary — they come from the **plant dynamics**.

**Step 1: Find the plant gain $G$.**
Linearising the angular dynamics around zero gimbal angle:

$$\dot{\omega}_y = \frac{M_y}{I_{yy}} = \frac{-L \, T \, \delta_y}{I_{yy}}$$

The controller commands $\delta_y = K_P \, e_y$, and for the closed
loop we substitute to get:

$$\dot{\omega}_y = -\frac{L \, T \, K_P}{I_{yy}} \, e_y$$

But $\dot{e}_y \approx \omega_y$ (for small angles), so the
closed-loop system is a second-order oscillator:

$$\ddot{e}_y + K_D \, G \, \dot{e}_y + K_P \, G \, e_y = 0$$

where

$$G = \frac{L \cdot F_\max}{I_{yy}}
  = \frac{2.913 \times 200\,000}{28\,583}
  \approx 20.4 \;\text{rad/s}^2\text{/rad}$$

**Step 2: Choose desired closed-loop behaviour.**
Compare with the standard 2nd-order form
$\ddot{e} + 2\zeta\omega_n\dot{e} + \omega_n^2 e = 0$:

$$K_P = \frac{\omega_n^2}{G}, \qquad
K_D = \frac{2\zeta\omega_n}{G}$$

Choosing **natural frequency** $\omega_n = 3\;\text{rad/s}$ and
**damping ratio** $\zeta = 0.85$:

$$K_P = \frac{9}{20.4} \approx 0.44 \quad (\text{used: } 0.5)$$
$$K_D = \frac{2 \times 0.85 \times 3}{20.4} \approx 0.25$$

These are the actual gains in `config.py` (`PID_KP_ATT = 0.5`,
`PID_KD_ATT = 0.25`).

**What do the numbers mean?**

- The inner loop has a bandwidth of ~3 rad/s ≈ 0.48 Hz: it can track
  attitude changes up to about twice per second.
- The damping of 0.85 means it converges quickly with almost no
  overshoot (critically damped is at 1.0).
- Gimbal saturation occurs at $e \approx \delta_\max / K_P
  = 7.5° / 0.5 \approx 15°$ of attitude error.

### 5.7 Roll Control via Ailerons

Roll (rotation about body-$z$) cannot be controlled by the gimbal
(the gimbal produces zero $M_z$).  Instead, **ailerons** (grid fins)
apply a direct body-frame torque.

$$\tau_z = \text{clip}\!\left(-K_{P,\text{roll}} \, e_z - K_{D,\text{roll}} \, \omega_z,\;\pm\tau_\max\right)$$

Note the **negative sign** here — standard negative feedback — because
the aileron torque has no geometric inversion (unlike the gimbal):
a positive torque command produces a positive moment.

**Roll gain derivation:**
The roll plant is $\dot{\omega}_z = \tau_z / I_{zz}$, where
$I_{zz} \approx 612\;\text{kg·m}^2$.  Targeting $\omega_{n,\text{roll}} = 3\;\text{rad/s}$:

$$K_{P,\text{roll}} = \omega_n^2 \cdot I_{zz} = 9 \times 612 = 5\,508 \approx 5\,500 \;\text{N·m/rad}$$
$$K_{D,\text{roll}} = 2\zeta \omega_n \cdot I_{zz} = 5.1 \times 612 = 3\,121 \approx 3\,300 \;\text{N·m·s/rad}$$

---

## 6. Phase 1b — Coast (Prograde Hold)

**File:** `src/controllers/ascent_pid.py` (method `_coast`)

After Main Engine Cut-Off (MECO) at $t = 80\;\text{s}$, the rocket
coasts upward on its momentum.  The engine is off (throttle = 0).
The same PD inner loop (§5.5) now commands only **ailerons** to keep
the body axis aligned with the velocity vector (**prograde**
direction).

$$\hat{d}_\text{des} = \frac{\mathbf{v}}{\|\mathbf{v}\|}$$

This alignment reduces aerodynamic drag and prepares the rocket for
the upcoming flip.

---

## 7. Phase 1c — MPC Flip Manoeuvre

**File:** `src/controllers/flip_mpc.py`

At apogee the velocity reverses direction.  The rocket must rotate 180°
so that the engine faces into the new velocity vector ("retrograde").
This large-angle manoeuvre is highly nonlinear — a PD loop designed
for small errors would perform poorly.  **Model Predictive Control**
(MPC) is ideal here because it can plan ahead and respect actuator
limits.

### 7.1 What Is Model Predictive Control?

MPC works in three steps, repeated every time-step:

1. **Predict** the system's behaviour over a future horizon using a
   *model* of the dynamics.
2. **Optimise** the control inputs over that horizon to minimise a
   *cost function*.
3. **Apply** only the first control input, then re-plan next step.

This "receding horizon" strategy makes MPC remarkably good at handling
constraints (e.g., gimbal limits) and nonlinear dynamics.

```
 now        ──────── prediction horizon (N steps) ────────→
  ├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
  ↑               optimise these inputs
  apply
  this one
```

### 7.2 Prediction Model

The MPC uses a **simplified rotational-only model** — translation is
unaffected by the 25 % throttle used during the flip.

For each prediction step ($\Delta t_\text{MPC} = 0.1\;\text{s}$):

1. Compute the thrust moment from the candidate gimbal angles
   (§3.5).
2. Apply Euler's rotation equation (§3.4) with one **Euler step**:

$$\boldsymbol{\omega}_{k+1} = \boldsymbol{\omega}_k + I^{-1}(\mathbf{M}_k - \boldsymbol{\omega}_k \times I\boldsymbol{\omega}_k) \, \Delta t$$

3. Propagate the quaternion:

$$q_{k+1} = \text{normalize}\!\left(q_k + \tfrac{1}{2} q_k \otimes [0, \boldsymbol{\omega}_k] \, \Delta t\right)$$

This is a first-order (Euler) integration — just accurate enough for
prediction, much cheaper than RK4.

### 7.3 Cost Function

The optimiser minimises:

$$J = w_\theta \, \theta_N^2 + w_\omega \, \|\boldsymbol{\omega}_N\|^2 + w_u \sum_{k=0}^{N-1} \left(\delta_{y,k}^2 + \delta_{z,k}^2\right)$$

| Term | Weight | Purpose |
|---|---|---|
| $\theta_N^2$ | $w_\theta = 100$ | Minimise the angle to retrograde at end of horizon |
| $\|\boldsymbol{\omega}_N\|^2$ | $w_\omega = 20$ | Arrive with low angular rate (don't spin wildly) |
| $\sum \delta^2$ | $w_u = 1$ | Penalise excessive gimbal use (smooth commands) |

$\theta_N$ is the shortest rotation angle between the predicted
terminal quaternion and the retrograde target quaternion:

$$\theta = 2 \, \arccos|q_0^\text{err}|$$

where $q^\text{err} = q_\text{target}^{*} \otimes q_\text{predicted}$.

### 7.4 Optimisation and Warm-Starting

The optimiser is **SLSQP** (Sequential Least Squares Quadratic
Programming) from `scipy.optimize`.  At each call it adjusts
$2N = 40$ variables (gimbal-Y and gimbal-Z for each of the $N = 20$
prediction steps) subject to box constraints
$\delta \in [-7.5°, +7.5°]$.

**Warm-starting:** The previous solution is shifted by one step and
used as the initial guess for the next call.  This dramatically
speeds up convergence — typically the optimiser needs only a few
iterations to refine the already-good guess.

---

## 8. Phase 2 — Sliding-Mode Controller (Ballistic Descent)

**File:** `src/controllers/ballistic_smc.py`

After the flip, the engine is off.  The rocket falls nose-first
through the atmosphere, and the **dynamic pressure** changes by
orders of magnitude as it descends.  A fixed-gain PD controller would
either be too weak at high altitude or too aggressive at low altitude.
**Sliding-Mode Control (SMC)** is inherently robust to such disturbances.

### 8.1 What Is Sliding-Mode Control?

SMC defines a **sliding surface** $\sigma = 0$ in the state space.
The control law drives the state onto this surface and keeps it there.
Once on the surface, the system's behaviour is governed by the surface
equation, regardless of disturbances.

Think of it like a marble in a groove: the controller "slams" the state
into the groove (the sliding surface), and once in the groove the
marble follows it to the origin regardless of bumps along the way.

### 8.2 The Sliding Surface

The sliding surface for each body axis is:

$$\sigma_i = \omega_i + \lambda \, e_{\text{att},i}$$

where:
- $\omega_i$ is the angular velocity about axis $i$ (we want it zero),
- $e_{\text{att},i}$ is the attitude error component (we want it zero),
- $\lambda = 2.0$ is the **surface slope**.

When $\sigma = 0$: $\omega = -\lambda \, e_\text{att}$, which means
the attitude error decays exponentially with time constant $1/\lambda = 0.5\;\text{s}$.

### 8.3 Control Law with Boundary Layer

The ideal sliding-mode law is:

$$\tau_i = -K \, \text{sign}(\sigma_i)$$

but the discontinuous `sign` function causes **chattering** — rapid
switching that excites high-frequency vibrations.  We replace it with
a **saturation function** (boundary layer):

$$\text{sat}(s, \Phi) = \begin{cases}
s / \Phi & \text{if } |s| < \Phi \\
\text{sign}(s) & \text{if } |s| \ge \Phi
\end{cases}$$

The smoothed control law is:

$$\tau_i = -K \cdot \text{sat}\!\left(\frac{\sigma_i}{\Phi}\right) \cdot \tau_\max \cdot s_\text{qdyn}$$

**Parameters:** $K = 5$ (switching gain), $\Phi = 0.1$ (boundary-layer
width in rad/s).

Inside the boundary layer ($|\sigma| < \Phi$), the controller
behaves like a proportional controller.  Outside, it saturates at
full authority — guaranteeing the state is driven back toward the
surface.

### 8.4 Dynamic-Pressure Scaling

Grid-fin torque authority depends on dynamic pressure — fins are
ineffective in thin air and very effective at sea level.  The factor:

$$s_\text{qdyn} = \text{clip}\!\left(\frac{q_\infty}{q_\text{ref}},\;0.05,\;1.0\right)$$

with $q_\text{ref} = 10\,000\;\text{Pa}$ scales the commanded torque
so the controller "knows" how much authority it actually has.

---

## 9. Phase 3 — LQR Landing Controller

**File:** `src/controllers/landing_lqr.py`

This is the most complex controller.  It orchestrates:

1. **Throttle scheduling** — a suicide-burn profile that decelerates
   the rocket from ~300 m/s to 0 m/s just as it reaches the ground.
2. **Attitude targeting** — pointing retrograde to kill velocity,
   blending to vertical for the final descent.
3. **LQR attitude tracking** — a mathematically optimal
   inner loop that maps attitude error to gimbal commands.
4. **Aileron assist** — grid-fin torques that supplement the gimbal
   when aerodynamic forces are large.

### 9.1 Suicide-Burn Throttle Logic

A **suicide burn** (also called a **hoverslam**) is the most
fuel-efficient way to land: fire the engine at full power as late as
possible, decelerating the rocket right down to zero velocity at
ground level.

The controller computes the required deceleration using basic
kinematics.  For the vertical channel:

$$a_\text{max} = \frac{F_\max}{m} - g \approx 10.19\;\text{m/s}^2$$

$$d_\text{stop} = \frac{v_\text{down}^2}{2 \, a_\text{max}}$$

The burn is triggered (in `phase_manager.py`) when
$\text{altitude} \le d_\text{stop} \times \text{margin}$.

Once burning, the controller accounts for **both vertical and
horizontal velocity**.  The total deceleration needed is the vector
sum:

$$a_\text{total} = \sqrt{a_{z}^2 + a_\text{hor}^2}$$

The throttle command is:

$$\text{throttle} = \text{clip}\!\left(\frac{m \cdot a_\text{total}}{F_\max},\;0,\;1\right)$$

A **descent bias** ($-1.5\;\text{m/s}^2$) is added when the rocket
is nearly hovering, so it does not stall at a fixed altitude but
continues gently descending to touchdown.

### 9.2 Retrograde-to-Vertical Attitude Blending

The rocket must point into its velocity vector to burn off all speed
components.  But once nearly stopped, it should point straight up for
a controlled vertical descent.

The **blending factor** is:

$$f = \text{clip}\!\left(\max\!\left(\frac{v_\text{hor}}{20},\;\frac{v_\text{total}}{50}\right),\;0,\;1\right)$$

$$\hat{d}_\text{des} = \text{normalize}\!\left(f \cdot \hat{d}_\text{retro} + (1 - f) \cdot \hat{d}_\text{vertical}\right)$$

- When fast ($f = 1$): purely retrograde — burns off all velocity.
- When slow ($f = 0$): purely vertical — clean final descent.
- In between: a smooth blend.

### 9.3 What Is LQR?

**LQR** (Linear-Quadratic Regulator) is a control technique that
finds the *mathematically optimal* feedback gain matrix $K$ for a
**linear** system.

Given a linear system:

$$\dot{\mathbf{x}} = A\,\mathbf{x} + B\,\mathbf{u}$$

LQR minimises the **infinite-horizon quadratic cost**:

$$J = \int_0^\infty \left( \mathbf{x}^\top Q \, \mathbf{x} + \mathbf{u}^\top R \, \mathbf{u} \right) dt$$

- $Q$ is a positive semi-definite matrix that **penalises state errors**
  (how much do we care about deviations?).
- $R$ is a positive definite matrix that **penalises control effort**
  (how expensive is it to use the actuators?).

The optimal control law is a **linear state feedback**:

$$\mathbf{u} = -K \, \mathbf{x}$$

where $K = R^{-1} B^\top P$ and $P$ is the solution of the
**Continuous Algebraic Riccati Equation** (CARE).

### 9.4 Linearised Plant Model (A and B Matrices)

The full 13-state nonlinear dynamics are too complex for
LQR.  We **linearise** around the current operating point (vertical
orientation, current thrust) to obtain a small 4-state system for the
pitch/yaw attitude tracking problem.

**States (4):** $\mathbf{x} = [e_x,\; e_y,\; \omega_x,\; \omega_y]$

These are the two attitude error components and the two angular rate
components (body $x$ and $y$ axes).  Roll is handled separately by
ailerons.

**Controls (2):** $\mathbf{u} = [\delta_y,\; \delta_z]$ (gimbal angles)

**A-matrix** (how the state evolves on its own):

$$A = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix}$$

Interpretation: $\dot{e}_x = \omega_x$ and $\dot{e}_y = \omega_y$
(attitude error changes at the angular rate).
The angular rates don't change on their own (zero in rows 3–4) —
they only change when a moment is applied.

**B-matrix** (how control inputs affect the state):

From §3.5, the linearised moments are:

$$\dot{\omega}_x = \frac{M_x}{I_{xx}} = \frac{-L \, T}{I_{xx}} \, \delta_z = -g_x \, \delta_z$$

$$\dot{\omega}_y = \frac{M_y}{I_{yy}} = \frac{-L \, T}{I_{yy}} \, \delta_y = -g_y \, \delta_y$$

where $g_x = g_y = L \cdot T / I \approx 20.4$ at full thrust.

$$B = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & -g_x \\
-g_y & 0
\end{bmatrix}$$

> The **cross-coupling** ($\delta_y$ affects $\omega_y$, not $\omega_x$, and
> vice versa) and the **both-negative signs** are critical.  Getting
> either wrong leads to the controller *amplifying* errors instead of
> correcting them.

**$g_x$ and $g_y$ are recalculated every step** based on the current
throttle, making this a **gain-scheduled** LQR — the gains adapt as
thrust changes during the burn.

### 9.5 The Riccati Equation

The LQR gain $K$ is computed by solving the **Continuous Algebraic
Riccati Equation** (CARE):

$$A^\top P + P A - P B R^{-1} B^\top P + Q = 0$$

This matrix equation is solved numerically by `scipy.linalg.solve_continuous_are`.
The gain is then:

$$K = R^{-1} B^\top P$$

**Weight matrices used:**

$$Q = \text{diag}(40,\;40,\;4,\;4), \qquad R = \text{diag}(1,\;1)$$

The larger values on $e_x$ and $e_y$ (40 vs 4) mean we care more
about reducing attitude error than about reducing angular rate —
the controller prioritises pointing accuracy.

Because the $B$-matrix changes with throttle (which increases as the
rocket decelerates harder), the Riccati equation is re-solved every
time-step.  This is computationally inexpensive for a 4 × 4 system.

### 9.6 Aileron Pitch / Yaw Assist

During descent, aerodynamic forces can overpower the gimbal (which
only has ±7.5° of travel).  **Grid-fin torques** supplement gimbal
control with a direct PD law (no sign inversion — positive torque
produces positive moment):

$$\tau_{\text{ail},x} = \text{clip}\!\left(-K_{P,\text{ail}} \, e_x - K_{D,\text{ail}} \, \omega_x,\;\pm\tau_\max\right)$$

$$\tau_{\text{ail},y} = \text{clip}\!\left(-K_{P,\text{ail}} \, e_y - K_{D,\text{ail}} \, \omega_y,\;\pm\tau_\max\right)$$

with $K_{P,\text{ail}} = 3000\;\text{N·m/rad}$ and
$K_{D,\text{ail}} = 1500\;\text{N·m·s/rad}$.

### 9.7 PD Fallback

If `scipy` is unavailable or the Riccati solve fails, the controller
falls back to a PD inner loop identical in structure to the ascent
controller (§5.5), with the same gains ($K_P = 0.5$, $K_D = 0.25$).

---

## 10. Phase Transitions — When Does the Phase Change?

**File:** `src/simulation/phase_manager.py`

| Transition | Condition | Physical Meaning |
|---|---|---|
| 1a → 1b | $t \ge 80\;\text{s}$ (MECO time) | Scheduled engine cut-off |
| 1b → 1c | $v_z \le 0$ and $\text{alt} > 1000\;\text{m}$ | Rocket has stopped climbing (apogee) |
| 1c → 2 | Angle to retrograde $< 10°$ **or** $\Delta t > 60\;\text{s}$ | Flip complete (or timed out) |
| 2 → 3 | $\text{alt} \le d_\text{stop} \times \text{margin}$ | Time to start braking (suicide-burn trigger) |
| 3 → landed | $\text{alt} \le 1\;\text{m}$ and $|v_z| \le 2\;\text{m/s}$ | On the ground, slow enough |

The **suicide-burn trigger** is the most interesting:

$$d_\text{stop} = \frac{v_\text{down}^2}{2 \, a_\max}$$

This is the minimum stopping distance from basic kinematics
($v^2 = u^2 + 2as$ with $v = 0$).  When the current altitude drops to
$d_\text{stop} \times \text{margin}$, it is time to light the engine or
the rocket will crater.  The `SUICIDE_BURN_MARGIN` is set to essentially
1.0 (minimum margin) for maximum fuel efficiency.

---

## 11. Reference Trajectory — Hermite-Parameterised Quarter-Ellipse

**File:** `src/models/trajectory.py`

Phase 1a tracks a pre-computed trajectory — a quarter-ellipse from
the launchpad $(0, 0, 0)$ to apogee $(X_\text{down},\;0,\;Z_\text{apogee})$
in the X-Z plane.

The parametric curve is:

$$x(t) = X \cdot (1 - \cos s(t))$$
$$z(t) = Z \cdot \sin s(t)$$

where $s \in [0, \pi/2]$.  The key is **how $s$ advances with time**.
A linear $s(t)$ would produce a sharp velocity at $t = 0$ and an
equally sharp stop at $t = T$, which the rocket cannot follow.  Instead
we use a **Hermite interpolation**:

$$\tau = \frac{t}{T_\text{coast}}, \qquad
f(\tau) = 3\tau^2 - 2\tau^3, \qquad
s = \frac{\pi}{2} \, f(\tau)$$

This S-curve satisfies $f(0) = 0$, $f(1) = 1$, **and**
$f'(0) = f'(1) = 0$ — meaning velocity is zero at both endpoints.
The rocket smoothly accelerates from the pad, reaches peak speed at
the midpoint, and smoothly decelerates to zero at apogee.

Velocity is obtained by differentiating:

$$\dot{s} = \frac{\pi}{2} \cdot \frac{f'(\tau)}{T_\text{coast}}, \qquad
f'(\tau) = 6\tau(1-\tau)$$

$$v_x = X \sin(s) \cdot \dot{s}, \qquad v_z = Z \cos(s) \cdot \dot{s}$$

---

## 12. Control-Vector Summary

Every controller returns a 6-element vector:

| Index | Name | Unit | Description |
|---|---|---|---|
| 0 | `throttle` | 0 – 1 | Fraction of max thrust (200 kN) |
| 1 | `gimbal_y` | rad | Nozzle deflection about body-Y |
| 2 | `gimbal_z` | rad | Nozzle deflection about body-X |
| 3 | `aileron_τx` | N·m | Direct torque about body-X |
| 4 | `aileron_τy` | N·m | Direct torque about body-Y |
| 5 | `aileron_τz` | N·m | Direct torque about body-Z (roll) |

**Which phases use which channels:**

|  | throttle | gimbal_y | gimbal_z | ail_x | ail_y | ail_z |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **1a** | ✓ | ✓ | ✓ | — | — | ✓ |
| **1b** | — | — | — | ✓ | ✓ | ✓ |
| **1c** | ✓ (25%) | ✓ | ✓ | — | — | ✓ |
| **2**  | — | — | — | ✓ | ✓ | ✓ |
| **3**  | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

---

## 13. Parameter Table

All values live in `src/config.py`.

### Rocket

| Parameter | Value | Symbol |
|---|---|---|
| Mass | 10 000 kg | $m$ |
| Length | 5.825 m | $L_\text{body}$ |
| CG from bottom | 2.913 m | $L$ (lever arm) |
| Diameter | 0.7 m | $d$ |
| Lateral inertia | ≈ 28 583 kg·m² | $I_{xx} = I_{yy}$ |
| Roll inertia | ≈ 612 kg·m² | $I_{zz}$ |

### Propulsion

| Parameter | Value |
|---|---|
| Max thrust $F_\max$ | 200 000 N |
| Max gimbal $\delta_\max$ | 7.5° (0.131 rad) |
| Gimbal slew rate | 20 °/s |
| Aileron max torque | 5 000 N·m |

### Aerodynamics

| Parameter | Value |
|---|---|
| Reference area $S_\text{ref}$ | $\pi r^2 \approx 0.385\;\text{m}^2$ |
| Lift slope $C_{L_\alpha}$ | 3.5 /rad |
| Zero-AoA drag $C_{D_0}$ | 0.3 |
| AoA drag increment $C_{D_\alpha}$ | 1.2 /rad² |
| CP offset (ascent) | +0.5 m |
| CP offset (descent) | −1.5 m |

### Controller Gains

| Gain | Value | Used In | Derivation |
|---|---|---|---|
| $K_{P,\text{pos}}$ | [0.3, 0.3, 0.5] | Ascent outer loop | Tuning |
| $K_{D,\text{vel}}$ | [1.5, 1.5, 2.5] | Ascent outer loop | Tuning |
| $K_{P,\text{att}}$ | 0.5 | Ascent/landing inner loop | $\omega_n^2 / G$ |
| $K_{D,\text{att}}$ | 0.25 | Ascent/landing inner loop | $2\zeta\omega_n / G$ |
| $K_{P,\text{roll}}$ | 5 500 N·m/rad | Roll axis | $\omega_n^2 \cdot I_{zz}$ |
| $K_{D,\text{roll}}$ | 3 300 N·m·s/rad | Roll axis | $2\zeta\omega_n \cdot I_{zz}$ |
| $\lambda_\text{SMC}$ | 2.0 | SMC surface slope | $1/\lambda$ = 0.5 s time constant |
| $K_\text{SMC}$ | 5.0 | SMC switching gain | — |
| $\Phi_\text{SMC}$ | 0.1 | SMC boundary layer | Chatter vs precision trade-off |
| MPC horizon $N$ | 20 steps × 0.1 s | Flip MPC | 2 s look-ahead |
| LQR $Q$ diag | [40, 40, 4, 4] | Landing inner loop | Attitude > rate priority |
| LQR $R$ diag | [1, 1] | Landing inner loop | Neutral actuator cost |

---

## 14. Glossary

| Term | Definition |
|---|---|
| **6-DOF** | Six Degrees of Freedom — 3 translational (x, y, z) + 3 rotational (roll, pitch, yaw) |
| **Body frame** | Coordinate system fixed to the rocket. $z_B$ = nose direction |
| **World frame** | Inertial (non-rotating) coordinate system. Z = up |
| **Quaternion** | 4-element representation of orientation; avoids gimbal lock |
| **DCM** | Direction Cosine Matrix — 3×3 rotation matrix equivalent of a quaternion |
| **Gimbal** | Mechanical degree of freedom that tilts the rocket nozzle |
| **Aileron / grid fin** | Aerodynamic control surface that produces torque proportional to dynamic pressure |
| **TVC** | Thrust Vector Control — steering by gimballing the nozzle |
| **CG** | Centre of Gravity — the point about which moments are computed |
| **CP** | Centre of Pressure — the point where aerodynamic force effectively acts |
| **AoA / α** | Angle of Attack — angle between velocity vector and body axis |
| **Dynamic pressure** | $q_\infty = \frac{1}{2}\rho v^2$ — proportional to aerodynamic forces |
| **Retrograde** | Direction opposite to velocity vector |
| **Prograde** | Direction of velocity vector |
| **MECO** | Main Engine Cut-Off |
| **Suicide burn / hoverslam** | Starting the deceleration burn at the last possible moment |
| **Bandwidth** | Frequency at which a controller can respond (higher = faster tracking) |
| **Natural frequency $\omega_n$** | Oscillation frequency of an undamped 2nd-order system |
| **Damping ratio $\zeta$** | 0 = undamped, 1 = critically damped, >1 = over-damped |
| **Plant gain $G$** | Output per unit input of the physical system (before controller gains) |
| **RK4** | 4th-order Runge-Kutta numerical integration method |
| **CARE** | Continuous Algebraic Riccati Equation — solved to obtain LQR gains |
| **SLSQP** | Sequential Least Squares Quadratic Programming — constrained optimiser |
| **Chattering** | High-frequency oscillation caused by discontinuous switching in SMC |
| **Warm-starting** | Initialising an optimiser with the (shifted) previous solution |
| **Gain scheduling** | Adjusting controller gains based on the current operating point |
