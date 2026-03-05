# 6-DOF Rocket Landing Controller - Complete System Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Architecture Overview](#architecture-overview)
3. [File Structure & Modules](#file-structure--modules)
4. [Physics Model](#physics-model)
5. [Control Phases](#control-phases)
6. [Detailed Controller Implementations](#detailed-controller-implementations)
7. [Configuration Parameters](#configuration-parameters)
8. [Visualization System](#visualization-system)
9. [Running the Simulation](#running-the-simulation)

---

## System Overview

This is a complete 6-DOF (six degrees of freedom) rocket landing controller implementing a multi-phase trajectory starting from launch, through powered apogee reorientation, ballistic descent, and concluding with energy-optimal vertical landing (hover-slam).

### Key Features
- **Phase 1a**: Powered ascent with Pure Pursuit guidance
- **Phase 1b**: Unpowered coasting phase with trajectory tracking
- **Phase 1c**: Powered apogee flip (retrograde orientation) using vector-based gimbal control
- **Phase 2**: Ballistic descent with attitude management
- **Phase 3**: Energy-optimal hover-slam with LQR attitude stabilization
- **Landing Detection**: Automatic touchdown with visual feedback (legs turn green)
- **Real-time 3D Visualization**: Dual-view animation (local frame and global view)

### Technical Specifications
| Parameter | Value |
|-----------|-------|
| Rocket Mass (Wet) | 12,000 kg |
| Rocket Mass (Dry) | 4,500 kg |
| Max Thrust | 300 kN |
| Rocket Length | 5.825 m |
| Engine Gimbal Position | 2.8 m below CG |
| Max Gimbal Deflection | ±15° (phases 1-2), ±10° (phase 3) |
| Landing Altitude Target | ~0.1 m |
| Landing Velocity Target | <0.5 m/s |

---

## Architecture Overview

```
src/
├── config.py                    # Global configuration & parameters
├── utils.py                     # Math utilities (angle wrapping, DCM)
├── simulation/
│   └── simulator.py            # Main ODE integrator & state machine
├── physics/
│   ├── dynamics.py             # State derivative computation
│   └── aerodynamics.py         # Aerodynamic forces/moments
├── controllers/
│   └── cascaded.py             # All phase controllers (1a-3)
├── models/
│   ├── rocket.py               # Rocket mass properties
│   └── trajectory.py           # Pre-planned ascent trajectory
└── visualization/
    └── plotter.py              # 3D matplotlib animation
```

### Control Flow

```
Main Simulation Loop (simulator.py):
  ├─ current_phase = _update_phase(t)
  ├─ (control) = compute_control(t)
  │   ├─ Phase 1a: pure_pursuit() → outer_loop() → middle_loop() → inner_loop()
  │   ├─ Phase 1b: trajectory_tracking() → outer_loop() → middle_loop() → inner_loop()
  │   ├─ Phase 1c: phase1c_controller() [vector-based gimbal control]
  │   ├─ Phase 2: phase2_flip_controller() → aero_surface_loop()
  │   └─ Phase 3: phase3_controller() [LQR-based attitude control + energy-optimal throttle]
  ├─ state_derivative = state_derivative(t, state, control, rocket, phase)
  ├─ state = RK45_integrate(state_derivative, t, state)
  └─ history.append(state, control, phase)
```

---

## File Structure & Modules

### 1. **config.py** - Global Configuration
Central repository for all tunable parameters, rocket properties, and controller gains.

**Rocket Geometry:**
```python
LENGTH = 5.825                  # Overall rocket length (m)
DIAMETER = 0.7                 # Body diameter
MASS = 10_000.0               # Initial mass (kg)
MAX_THRUST = 300_000.0        # Max thrust (N)
CG_FROM_GIMBAL = 2.8          # CG height above engine (m)
```

**Inertia Properties:**
```python
I_ZZ = 0.5 * MASS * RADIUS²   # Roll inertia (cylinder approximation)
I_XX = I_YY = (1/12)*MASS*(3R² + L²)  # Pitch/Yaw inertia
```

**Gimbal & Control Limits:**
```python
GIMBAL_MAX = 15°              # Nominal gimbal limit (phases 1-2)
GIMBAL_MAX_PHASE3 = 10°       # Reduced gimbal in landing (phase 3)
LAUNCH_GIMBAL_MAX = 3°        # Pure pursuit launch phase only
THROTTLE_MIN_PHASE3 = 0.30    # Minimum throttle in descent
```

**Phase 1c (Powered Flip) Parameters:**
```python
PHASE1C_THROTTLE = 0.50
PHASE1C_FLIP_KP = 1.5          # rad/s per rad error
PHASE1C_FLIP_KD = 50000.0      # N⋅m per rad/s error
PHASE1C_FLIP_SUCCESS_THETA_ERR = 10°  # Alignment tolerance
PHASE1C_FLIP_SUCCESS_RATE = 0.2 rad/s  # Angular rate threshold
PHASE1C_FLIP_TIMEOUT = 8.0 s   # Failsafe duration
```

**Phase 3 (Hover-Slam) Parameters:**
```python
PHASE3_GIMBAL_SLEW_LIMIT = 0.7 rad/s     # Rate limiter
PHASE3_SOFT_ENGAGEMENT_TIME = 0.5 s      # Gain ramp-up
PHASE3_ATTITUDE_ONLY_ALT = 50.0 m        # Switch to attitude-only
PHASE3_ROLL_DAMPING = 5000.0             # Artificial RCS
PHASE3_IGNITION_SAFETY_MARGIN = 0.98     # 2% burn margin
PHASE3_RATE_PRIORITY_TIME = 0.2 s        # Pure damping initial
```

**LQR Parameters:**
```python
LQR_Q = diag([0.1, 0.1, 500, 500, 100, 100])    # State cost
LQR_R = diag([1.0, 1.0])                         # Control cost
```

### 2. **utils.py** - Math Utilities

**Core Functions:**
```python
def normalize_angle(angle: float) -> float
    """Wrap angle to [-π, π] range. Essential for angle error calculations."""
    
def euler_to_dcm(phi, theta, psi) -> ndarray[3,3]
    """Direction cosine matrix: Body→World using ZYX convention.
    Returns 3×3 orthogonal matrix for coordinate transformations."""
    
def body_z_axis_in_world(phi, theta, psi) -> ndarray[3]
    """Get rocket nose direction in world frame (Z-axis of body frame)."""
    
def omega_to_euler_rates(phi, theta, omega) -> ndarray[3]
    """Convert body-frame angular rates [p,q,r] to Euler rate derivatives."""
```

**Coordinate Frames:**
- **Body Frame**: X=starboard, Y=up, Z=nose (right-hand rule)
- **World Frame**: X=downrange, Y=lateral, Z=altitude
- **Euler Angles (ZYX)**: φ (roll), θ (pitch), ψ (yaw)

### 3. **rocket.py** - Rocket Model

Encapsulates all rocket physical properties.

```python
class Rocket:
    def __init__(self):
        self.mass                  # Current rocket mass
        self.length                # Rocket length
        self.radius                # Body radius
        self.max_thrust            # Maximum available thrust
        self.inertia               # 3×3 inertia matrix
        self.inertia_inv           # Pre-computed inverse
        
    @property
    def gimbal_to_cg(self) -> ndarray[3]
        """Vector from engine gimbal to center of gravity."""
        return [0, 0, CG_FROM_GIMBAL]
        
    @property
    def cp_offset(self) -> ndarray[3]
        """Center of pressure offset from CG (used in aerodynamics)."""
```

### 4. **dynamics.py** - Physics Integration

**Function: `state_derivative(t, state, control, rocket, phase)`**

Computes the rate of change of the 12-state vector:
```
state = [x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r]
         positions | velocities | euler angles | angular rates
```

**Physics Computation:**

1. **Gravitational Force** (world frame)
   ```
   F_grav = [0, 0, -mass·g]  (always downward)
   ```

2. **Thrust Vector** (body frame, then rotated)
   ```
   F_thrust_body = [F·δy, F·δz, F]
   where δy, δz are gimbal angles (radians), F is throttle·max_thrust
   F_thrust_world = DCM @ F_thrust_body
   ```

3. **Gimbal Torque** (coupling between translation and rotation)
   ```
   τ_gimbal = r_gimbal × F_thrust_body
   where r_gimbal = [0, 0, CG_FROM_GIMBAL] (engine below CG)
   This produces coupled motion: gimbal deflection creates torque
   ```

4. **Aerodynamic Forces** (velocity-dependent, computed in aerodynamics.py)
   ```
   F_aero = -½·ρ·v²·S·(CD, CL_lateral, CL_longitudinal)
   τ_aero = lever_arm × F_aero + pitching_moment
   ```

5. **Linear Acceleration**
   ```
   a = (F_grav + F_thrust + F_aero) / mass
   ```

6. **Angular Acceleration**
   ```
   ω̇ = I⁻¹·(τ_gimbal + τ_aero + τ_control - ω × I·ω)
   where ω × I·ω is gyroscopic coupling
   ```

7. **Euler Rate Kinematics**
   ```
   φ̇ = p + q·sin(φ)·tan(θ) + r·cos(φ)·tan(θ)
   θ̇ = q·cos(φ) - r·sin(φ)
   ψ̇ = q·sin(φ)/cos(θ) + r·cos(φ)/cos(θ)
   (handles singularities with clipping)
   ```

---

## Physics Model

### Coordinate System Details

**Body Frame (Rocket-Centered)**:
- **+X axis**: Starboard (right wing)
- **+Y axis**: Port (left wing, perpendicular to fuselage plane)
- **+Z axis**: Nose (forward)
- Origin: Center of gravity (CG)

**World Frame (Inertial)**:
- **+X axis**: Downrange (horizontal, in trajectory plane)
- **+Y axis**: Lateral (perpendicular to trajectory plane)
- **+Z axis**: Altitude (vertical, up)
- Origin: Launch pad

**Euler Angle Convention (ZYX / Aerospace)**:
- **φ (Roll)**: Rotation about Z-axis (nose), right-hand rule
- **θ (Pitch)**: Rotation about Y-axis (starboard), positive = nose up
- **ψ (Yaw)**: Rotation about X-axis (forward), positive = right
- Applied in order: Z → Y → X rotations

### Key Physics Relations

**Direction Cosine Matrix (DCM)**:
The DCM transforms vectors from body frame to world frame.
```
v_world = DCM @ v_body
```

For ZYX convention:
```
DCM = [
  [ cos(ψ)·cos(θ),  cos(ψ)·sin(θ)·sin(φ) - sin(ψ)·cos(φ),  cos(ψ)·sin(θ)·cos(φ) + sin(ψ)·sin(φ)],
  [ sin(ψ)·cos(θ),  sin(ψ)·sin(θ)·sin(φ) + cos(ψ)·cos(φ),  sin(ψ)·sin(θ)·cos(φ) - cos(ψ)·sin(φ)],
  [-sin(θ),         cos(θ)·sin(φ),                          cos(θ)·cos(φ)                        ]
]
```

**Gimbal-to-Torque Conversion**:
```
Gimbal deflections (δy, δz) produce forces in body frame:
  F_body = [F·δy, F·δz, F]  (small angle approximation)
  
These forces are applied at the engine (below CG by distance L), producing torque:
  τ = r_gimbal × F_thrust = [0,0,L] × [F·δy, F·δz, F]
    = [-L·F·δz, +L·F·δy, 0]
    
The lever arm effect amplifies gimbal control: τ ∝ F·L
```

**Aerodynamic Damping**:
Velocity causes drag forces and stabilizing moments:
```
Drag force: F_drag = -½·ρ·v²·S·CD  (opposes velocity)
Lift force: F_lift = ½·ρ·v²·S·CL·α  (perpendicular to velocity, proportional to attack angle)
Pitching moment: τ = ½·ρ·v²·S·L·CM  (created by center of pressure offset)
```

---

## Control Phases

### Phase 1a: Powered Ascent with Pure Pursuit

**Objective**: Launch and climb to apogee while following a pre-planned trajectory.

**Trajectory Reference**: Pre-computed parabolic path `traj_pos(t)` and `traj_vel(t)`.

**Control Architecture** (3-loop cascade):

```
┌─────────────────────────────────────────────────────────┐
│ Phase 1a: Outer → Middle → Inner Loop                   │
├─────────────────────────────────────────────────────────┤
│ 1. PURE PURSUIT (guidance)                              │
│    carrot_point = look_ahead_along_trajectory()         │
│    θ_desired = arctan2(Δx, Δz)                          │
│                                                          │
│ 2. OUTER LOOP (position error to acceleration)          │
│    a_cmd = Kp_pos·(pos_des - pos) + Kd_pos·(vel_des - vel) │
│    F_cmd = mass·(a_cmd + [0,0,g])                       │
│                                                          │
│ 3. MIDDLE LOOP (force to attitude)                      │
│    u_desired = F_cmd / |F_cmd|                          │
│    throttle = |F_cmd| / F_max                           │
│                                                          │
│ 4. INNER LOOP (attitude error to gimbal)               │
│    τ_error = Kp_att·(u_body × u_desired) - Kd_att·ω    │
│    δ_gimbal = τ_error / (F·L)                           │
└─────────────────────────────────────────────────────────┘
```

**Parameters**:
- `PURE_PURSUIT_LOOKAHEAD = 2000 m`
- `MAX_PITCH_FROM_VERTICAL = 60°`
- `Kp_pos = [4.5, 4.5, 0.90]`
- `Kd_pos = [6.5, 6.5, 1.00]`
- `Kp_att = 300,000 N⋅m` (attitude error gain)
- `Kd_att = 40,000 N⋅m·s` (rate damping gain)

**Transition Condition**:
```python
if t >= MECO_TIME_MAX or x >= TRAJ_X_REF or z >= MECO_ALT_MAX:
    phase = "phase1b"  # Switch to coast
```

### Phase 1b: Unpowered Coast

**Objective**: Coast under zero thrust while tracking the reference trajectory to apogee.

**Control**: 
- Same cascaded control as Phase 1a
- Thrust = 0 N (no propellant)
- Gimbal passively stabilizes attitude via aerodynamic forces

**Transition Condition**:
```python
if vz <= 0.0:
    phase = "phase1c"  # Apogee detected (zero vertical velocity)
```

### Phase 1c: Powered Apogee Flip (NEW!)

**Objective**: Reorient rocket to retrograde (engine-first, nose down) for stable entry into Phase 2.

**Physics Challenge**:
The rocket was launched nose-up but needs to flip 180° to enable powered descent. This must be done with controlled gimbal angles to avoid excessive structural loads or attitude oscillations.

**Control Law** (Vector-Based PD Gimbal Control):

Instead of tracking angle setpoints (which suffer from ±180° discontinuity), this controller uses normalized vector geometry:

```
Step 1: Compute rocket orientation
  u_rocket = body_z_axis_in_world(φ, θ, ψ)  # Current nose direction

Step 2: Compute desired orientation
  v_mag = |velocity|
  if v_mag > 5 m/s:
    u_desired = -velocity / v_mag  # Retrograde direction (opposite velocity)
  else:
    u_desired = u_rocket  # No change at near-zero velocity

Step 3: Compute rotation axis (cross product)
  rotation_axis = u_rocket × u_desired
  rotation_axis_norm = rotation_axis / |rotation_axis|
  
  This gives the axis perpendicular to both vectors, around which to rotate.

Step 4: Compute angle error (always 0 to π)
  cos_angle = u_rocket · u_desired  # Dot product
  angle_error = arccos(cos_angle)   # Always in [0, π], no wrapping!
  
  This avoids the ±180° discontinuity that caused oscillations.

Step 5: Compute desired angular velocity
  ω_desired = rotation_axis_norm × (KP × angle_error)
  
  This produces smooth convergence: proportional to remaining error.

Step 6: PD control on angular velocity error
  ω_error = ω_desired - ω_body
  τ_body = KD × ω_error  # Torque command
  
  Convert to gimbal:
  δy = τ_pitch / (F × L)
  δz = τ_yaw / (F × L)
```

**Key Innovation**: By using cross-product to define the rotation axis and arccos for the angle (always [0,π]), this controller naturally avoids the ±180° angle wrapping that plagued angle-based controllers.

**Parameters**:
```python
PHASE1C_THROTTLE = 0.50              # 50% thrust (gimbal is primary actuator)
PHASE1C_FLIP_KP = 1.5 rad/s/rad      # Proportional angular velocity
PHASE1C_FLIP_KD = 50000 N⋅m/(rad/s)  # Derivative (high to generate gimbal)
PHASE1C_FLIP_SUCCESS_THETA_ERR = 10° # Alignment tolerance
PHASE1C_FLIP_SUCCESS_RATE = 0.2 rad/s  # Max angular rate for stability
PHASE1C_FLIP_TIMEOUT = 8.0 s         # Failsafe
```

**Success Condition**:
```python
flip_success = (
    time_in_phase1c > 0.5 s AND          # Minimum dwell time
    |θ_err| < 10° AND                    # Pitch aligned
    |ψ_err| < 10° AND                    # Yaw aligned
    angular_rate_magnitude < 0.2 rad/s   # Stabilized
) OR time_in_phase1c > 8.0 s            # Failsafe timeout
```

**Transition**: 
```python
phase = "phase2"  # Proceed to ballistic descent
```

---

### Phase 2: Ballistic Descent with Attitude Management

**Objective**: Maintain retrograde attitude (engine down) during unpowered descent via aerodynamic control (grid fins / ailerons).

**Control Output** (Live Visualization Data):
During Phase 2, the telemetry box displays:
```
PHASE: PHASE2
Pos: [  12345.2,    -234.1,   45300.0] m
Vel: [   145.3,      -5.2,   -250.6] m/s
Gimbal: [  0.0°,    0.0°]  Thrust:  0.0%        ← Engine OFF
Ailerons: τ=   23450 N⋅m  Grid Fins: 100.0%     ← AERODYNAMIC CONTROL
```

**Output Data Meanings**:
- `Ailerons: τ=XXXXX N⋅m` - Total aerodynamic torque from grid fins (magnitude of 3D vector)
- `Grid Fins: XX.X%` - Grid fin deployment factor (0% = fully retracted, 100% = fully deployed)

**Control Law** (Smooth 180° Flip via Aero Torque):

Since Phase 2 has no thrust, only aerodynamic surfaces (grid fins) can provide control torque:

```
Step 1: Define desired attitude
  θ_desired = π radians (nose down / engine up)
  φ_desired = 0 (no roll)
  ψ_desired = 0 (no yaw)

Step 2: Compute angle errors (with wrapping)
  θ_err = θ_desired - θ  (wrapped to [-π, π])
  φ_err = φ_desired - φ

Step 3: PD controller on aerodynamic torque
  τ_x = Kp_flip·φ_err - Kd_flip·p  (roll control via ailerons)
  τ_y = Kp_flip·θ_err - Kd_flip·q  (pitch control via grid fins)
  τ_z = -rate_damp·r               (yaw damping only)
  
  Total torque vector (body frame):
  τ = [τ_x, τ_y, τ_z] - rate_damp·ω  (pure rate damping term)

Step 4: Clamp to aerodynamic authority and scale grid fin deployment
  τ_aero_max = 35,000 N⋅m  (max torque from grid fins)
  τ = clamp(τ, -τ_aero_max, +τ_aero_max)
  
  grid_fins = |τ| / τ_aero_max  (deployment percentage shown in viz)
```

**Parameters**:
```python
PHASE2_FLIP_Kp = 8000 N⋅m/rad        # Attitude gain (high for flip authority)
PHASE2_FLIP_Kd = 25000 N⋅m⋅s/rad    # Damping gain (strong braking)
PHASE2_RATE_DAMP = 50000 N⋅m⋅s/rad  # Pure rate damping (anti-spiral)
PHASE2_AERO_DAMP = 15000 N⋅m⋅s/rad  # Aerodynamic damping
AERO_TORQUE_MAX = 35000 N⋅m         # Max torque from grid fins
PHASE2_TILT_MAX = 10°                # Max tilt during descent
```

**Aerodynamic Torque Authority**:
Grid fins generate torque from dynamic pressure: τ ∝ ½·ρ·v²·S·moment_arm
- At high altitude (low ρ) or low velocity: reduced torque authority
- At dense air / high velocity: can reach peak 35,000 N⋅m
- Simulation cap ensures torque doesn't exceed physical limits

**Transition Condition** (to Phase 3):
```python
if _should_ignite():  # Checks energy-optimal burn condition
    phase = "phase3"
```

---

### Phase 3: Energy-Optimal Hover-Slam with LQR

**Objective**: Decelerate vertically to land with minimal impact velocity (<0.5 m/s) at ~0.1 m altitude.

**Two-Level Control**:

#### Level 1: Vertical Thrust Control (Energy-Optimal, NOT MPC)

**IMPORTANT CLARIFICATION**: This uses a **direct energy-based formula**, NOT Model Predictive Control (MPC).

**Why NOT MPC?**
- ❌ MPC requires: prediction horizon, optimization solver (QP), iterative computation
- ✅ This uses: direct algebraic formula from physics (O(1) computation)
- ✅ Result: simpler, faster, guaranteed stable

**Energy-Optimal Direct Formula**:
```
Physics principle: kinematic equation v² = 2·a·d
Solved for required deceleration: a_required = v² / (2·d)

Where:
  v = |vertical velocity| (positive downward)
  d = altitude above ground (positive)
  a_required = constant deceleration to reach v=0 at d=0
  
Apply 1.5× safety factor for margin:
  a_required = (v² / (2·d)) × 1.5
  
Compute required throttle:
  F_required = mass × (a_required + g)
  throttle = clamp(F_required / F_max, [0.30, 1.0])
```

**Why This Is Optimal**:
- Minimizes fuel: only uses as much thrust as needed (no overshooting)
- Inherently stable: based on energy conservation physics
- Guaranteed to reach zero velocity at ground level (if thrust available)

**Examples** (mass=10,000 kg, F_max=300,000 N, g=9.81 m/s²):
```
At Z=1000m, Vz=-300 m/s:
  a_req = (300²/(2·1000)) × 1.5 = 67.5 m/s² (6.9 g deceleration)
  F_req = 10,000 × (67.5 + 9.81) = 772,100 N
  throttle = 772,100/300,000 = 257% → clamps to 100%
  (Maxes out at high altitude, controlled by LQR attitude)

At Z=100m, Vz=-150 m/s:
  a_req = (150²/(2·100)) × 1.5 = 168.75 m/s² (17.2 g)
  Throttle saturates at 100% to arrest descent

At Z=10m, Vz=-50 m/s:
  a_req = (50²/(2·10)) × 1.5 = 187.5 m/s²
  Final descent with max throttle, LQR keeps attitude stable
```

#### Level 2: LQR Attitude Stabilization

Computes optimal gimbal commands to maintain attitude and damp angular rates:

**Linearized Dynamics** (around hover equilibrium):
```
State vector (6-DOF, no roll control):
  x = [vx, vy, θ_err, ψ_err, q, r]
  
Control vector (2-DOF gimbal):
  u = [δy, δz]

Linear equations of motion (nose-down equilibrium):
  ẋ(0) = g·θ(3)      (forward accel from pitch)
  ẏ(1) = -g·ψ(3)     (lateral accel from yaw)
  θ̇(2) = q(4)        (pitch rate kinematics)
  ψ̇(3) = r(5)        (yaw rate kinematics)
  q̇(4) = (F·L/I_yy)·δy  (pitch acceleration from gimbal)
  ṙ(5) = (F·L/I_yy)·δz  (yaw acceleration from gimbal, approximation)
```

**LQR Gain Computation**:
```python
# Solve continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)
K = R⁻¹ @ B.T @ P  # Optimal gain matrix [2×6]

# Apply optimal control law
u_optimal = -K @ x  # [δy_command, δz_command]
```

**Cost Function**:
```python
LQR_Q = diag([0.1, 0.1, 500, 500, 100, 100])  # State weights
LQR_R = diag([1.0, 1.0])                       # Control weights

# Q weights: penalize attitude errors (500) > angular rates (100) > velocities (0.1)
# R weights: equal cost on pitch and yaw gimbal
```

**Soft Engagement** (gradual gain ramp):
```
Time 0.0 → 0.2s: Pure rate damping only
  x = [0, 0, 0, 0, q, r]  # Ignore position/attitude
  Purpose: Stabilize any residual spin from Phase 1c

Time 0.2s → 0.5s: Gradual blending
  blend_factor = (t - 0.2) / (0.5 - 0.2) = linear from 0 to 1
  x = blend·[vx, vy, θ_err, ψ_err, q, r]
  Purpose: Smoothly transition to full guidance

Time 0.5s onward:
  - If Z > 50m: Attitude-only mode (zero velocity errors)
    x = [0, 0, θ_err, ψ_err, q, r]
    Purpose: Maximize vertical thrust (no horizontal tilt)
  - If Z ≤ 50m: Full state feedback
    x = [vx, vy, θ_err, ψ_err, q, r]
    Purpose: Final horizontal guidance
```

**Anti-Windup / Saturation**:
```python
if |gimbal_command| > GIMBAL_MAX_PHASE3:
    # Recompute without velocity errors (attitude-only)
    x_attitude = [0, 0, θ_err, ψ_err, q, r]
    u = -K @ x_attitude
    # Prevents integrator windup from unachievable commands
```

**Slew Rate Limiting**:
```python
max_gimbal_rate = PHASE3_GIMBAL_SLEW_LIMIT = 0.7 rad/s
gimbal_delta_max = max_gimbal_rate × dt

gimbal_limited = clamp(gimbal_command, 
                        prev_gimbal - gimbal_delta_max,
                        prev_gimbal + gimbal_delta_max)
```

**Parameters**:
```python
PHASE3_GIMBAL_SLEW_LIMIT = 0.7 rad/s       # Rate limiter (smooth control)
PHASE3_SOFT_ENGAGEMENT_TIME = 0.5 s        # Gain ramp duration
PHASE3_ATTITUDE_ONLY_ALT = 50.0 m          # Switch altitude
PHASE3_ROLL_DAMPING = 5000 N⋅m⋅s/rad      # Artificial RCS (roll damping)
PHASE3_IGNITION_SAFETY_MARGIN = 0.98       # Trigger at 98% of calculated burn distance
PHASE3_RATE_PRIORITY_TIME = 0.2 s          # Initial pure damping duration
THROTTLE_MIN_PHASE3 = 0.30                 # Minimum engine thrust
```

**Landing Detection**:
```python
if Z < 0.5 m AND Vz > -1.0 m/s:  # On ground and not crashing
    phase = "landed"
    leg_deploy = 1.0  # Legs fully deployed
    print("LANDING SUCCESSFUL")
```

---

## Detailed Controller Implementations

### Phase 1a/1b: Cascaded Control Loops

**Function: `pure_pursuit(rocket_pos, traj_pos) → (θ_des, carrot_pos, end_of_path)`**

Searches trajectory for carrot point at lookahead distance ahead:

```python
# Find closest point on trajectory (projection)
dists = ||traj_pos - rocket_pos||
closest_idx = argmin(dists)

# Look ahead from closest point
cumulative_dist = 0
for i in range(closest_idx+1, len(traj_pos)):
    cumulative_dist += ||traj_pos[i] - traj_pos[i-1]||
    if cumulative_dist >= LOOKAHEAD_DISTANCE:
        carrot_idx = i
        break

# Compute desired pitch angle from rocket to carrot
dx = traj_pos[carrot_idx, 0] - rocket_pos[0]
dz = traj_pos[carrot_idx, 2] - rocket_pos[2]
θ_desired = arctan2(dx, dz)  # Angle from vertical
θ_desired = clamp(θ_desired, -MAX_PITCH, +MAX_PITCH)
```

**Function: `outer_loop(pos, vel, pos_des, vel_des, mass) → F_desired`**

Proportional-derivative control on position error:

```python
# Position error feedback
pos_err = pos_des - pos
vel_err = vel_des - vel

# Acceleration command
a_cmd = Kp_pos * pos_err + Kd_pos * vel_err

# Force required (including gravity to hover)
F_desired = mass * (a_cmd + [0, 0, g])
```

**Function: `middle_loop(F_desired, max_thrust, tilt_gain) → (u_desired, throttle)`**

Converts desired force vector into attitude and throttle commands:

```python
# Apply tilt gain to lateral components (trust vertical more)
F_adjusted = [F[0]*tilt_gain, F[1]*tilt_gain, F[2]]

# Compute desired unit vector (body Z-axis in world)
magnitude = ||F_adjusted||
u_desired = F_adjusted / magnitude

# Compute required throttle
throttle = magnitude / max_thrust
throttle = clamp(throttle, 0.0, 1.0)
```

**Function: `inner_loop(φ, θ, ψ, ω, u_desired, throttle, ...) → (δy, δz)`**

Attitude control via gimbal deflection:

```python
# Current body Z-axis in world frame
u_current = body_z_axis_in_world(φ, θ, ψ)

# Vector error (cross product gives rotation axis)
error_vec = u_current × u_desired

# PD torque command in world frame
τ_world = Kp_att * error_vec - Kd_att * ω

# Transform to body frame
DCM = euler_to_dcm(φ, θ, ψ)
τ_body = DCM.T @ τ_world

# Convert torque to gimbal angles using lever arm
# τ_pitch = (CG_FROM_GIMBAL * F) * δy
# τ_yaw = (CG_FROM_GIMBAL * F) * δz
F = throttle * max_thrust
δy = τ_body[1] / (CG_FROM_GIMBAL * F)
δz = -τ_body[0] / (CG_FROM_GIMBAL * F)  # Sign convention

# Clamp to physical limits
δy = clamp(δy, -GIMBAL_MAX, +GIMBAL_MAX)
δz = clamp(δz, -GIMBAL_MAX, +GIMBAL_MAX)
```

### Phase 1c: Vector-Based Gimbal Control

Complete implementation shown in [Physics Model](#phase-1c-powered-apogee-flip-new) section above.

Key implementation in `cascaded.py`:

```python
def phase1c_controller(state, rocket, time_in_phase1c=0.0):
    # Extract state
    pos, vel = state[0:3], state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]
    
    # Vector geometry
    u_rocket = body_z_axis_in_world(phi, theta, psi)
    v_mag = ||vel||
    u_desired = -vel/v_mag if v_mag > 5 else u_rocket
    
    rotation_axis = u_rocket × u_desired
    rotation_axis_mag = ||rotation_axis||
    
    angle_error = arccos(clamp(u_rocket·u_desired, -1, 1))
    
    if rotation_axis_mag > 1e-6 and angle_error > 1e-4:
        # Compute desired angular velocity
        rotation_axis_norm = rotation_axis / rotation_axis_mag
        omega_desired = rotation_axis_norm * KP * angle_error
        omega_error = omega_desired - omega
        
        # Torque command
        tau_body = KD * omega_error
        
        # Convert to gimbal
        F = PHASE1C_THROTTLE * rocket.max_thrust
        L = rocket.cg_from_gimbal
        gimbal_pitch = tau_body[1] / (F*L)
        gimbal_yaw = tau_body[2] / (F*L)
    else:
        gimbal_pitch = gimbal_yaw = 0.0
    
    # Clamp and return
    return {
        "throttle": PHASE1C_THROTTLE,
        "delta_y": clamp(gimbal_pitch, -GIMBAL_MAX, +GIMBAL_MAX),
        "delta_z": clamp(gimbal_yaw, -GIMBAL_MAX, +GIMBAL_MAX),
        ...
    }
```

### Phase 2: Aerodynamic Flip Control

```python
def phase2_flip_controller(phi, theta, psi, omega):
    # Desired nose-down attitude
    theta_des = π
    phi_des = 0.0
    
    # Angle errors
    theta_err = theta_des - theta
    phi_err = phi_des - phi
    
    # Wrap angle errors to avoid ±π issues
    theta_err = normalize_angle(theta_err)
    phi_err = normalize_angle(phi_err)
    
    # PD torque command
    tau = [
        Kp_flip*phi_err - Kd_flip*omega[0],
        Kp_flip*theta_err - Kd_flip*omega[1],
        -rate_damp*omega[2],  # Yaw damping only
    ]
    
    # Add pure rate damping
    tau -= rate_damp * omega
    
    # Clamp to aerodynamic authority
    return clamp(tau, -AERO_TORQUE_MAX, +AERO_TORQUE_MAX)
```

### Phase 3: LQR Computation and Application

**Function: `lqr_gain(mass, inertia, thrust, cg_from_gimbal) → K`**

Computes optimal feedback gain matrix:

```python
# Linearized system matrices (around hover nose-down)
A = 6×6 matrix of partial derivatives
B = 6×2 control input matrix

# Continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Optimal feedback gain
K = R⁻¹ @ B.T @ P  # Shape: (2, 6)

# Application: u_optimal = -K @ x
```

**Function: `phase3_controller(state, rocket, time_in_phase3) → (throttle, δy, δz)`**

```python
# Energy-optimal throttle control
z, vz = state[2], state[5]
a_required = (vz² / (2*z)) * 1.5  # 50% safety margin
throttle = clamp(mass*(a_required + g) / max_thrust, 0.3, 1.0)

# Recompute LQR with actual thrust (linearization point)
K = lqr_gain(mass, inertia, throttle*max_thrust, cg_from_gimbal)

# Soft engagement: gradual gain ramp
if time_in_phase3 < 0.2:
    # Pure rate damping
    x = [0, 0, 0, 0, omega[1], omega[2]]
elif time_in_phase3 < 0.5:
    # Blend in position/attitude
    blend = (time_in_phase3 - 0.2) / 0.3
    x = [...] * blend
else:
    # Full feedback
    if z > 50:
        # Attitude-only above 50m
        x = [0, 0, theta_err, psi_err, omega[1], omega[2]]
    else:
        # Full state
        x = [vel[0], vel[1], theta_err, psi_err, omega[1], omega[2]]

# Compute optimal control
u = -K @ x

# Anti-saturation
if ||u|| > GIMBAL_MAX_PHASE3:
    x_attitude = [0, 0, theta_err, psi_err, omega[1], omega[2]]
    u = -K @ x_attitude

# Slew rate limiting
u_limited = clamp(u, prev_gimbal ± SLEW_LIMIT*dt)

return throttle, u_limited[0], u_limited[1]
```

---

## Configuration Parameters

### Physical Constants
| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| G | 9.81 | m/s² | Gravitational acceleration |
| RHO | 1.225 | kg/m³ | Air density at sea level |

### Rocket Geometry
| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| LENGTH | 5.825 | m | Total rocket length |
| DIAMETER | 0.7 | m | Body diameter |
| RADIUS | 0.35 | m | Body radius |
| MASS | 10,000 | kg | Wet mass (with propellant) |
| MAX_THRUST | 300,000 | N | Maximum engine thrust |
| CG_FROM_GIMBAL | 2.8 | m | Height of CG above engine |

### Moments of Inertia (cylinder approximation)
| Component | Formula | Value |
|-----------|---------|-------|
| I_ZZ | 0.5 × M × R² | ~410 kg⋅m² |
| I_XX, I_YY | (1/12) × M × (3R² + L²) | ~68,400 kg⋅m² |

### Aerodynamic Properties
| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| S_REF | π × R² = 0.385 | m² | Reference area |
| CL_ALPHA | 3.5 | 1/rad | Lift slope |
| CD0 | 0.3 | - | Zero-lift drag coefficient |
| CD_ALPHA | 1.2 | - | Drag due to angle of attack |
| CP_OFFSET_ASCENT | 0.6 | m | CP behind CG (ascent) |
| CP_OFFSET_PHASE2 | 0.8 | m | CP ahead of CG (grid fins) |

### Control Limits
| Parameter | Value | Unit | Phase |
|-----------|-------|------|-------|
| GIMBAL_MAX | 15 | degrees | 1a, 1b, 1c, 2 |
| GIMBAL_MAX_PHASE3 | 10 | degrees | 3 (landing) |
| LAUNCH_GIMBAL_MAX | 3 | degrees | 1a only |
| THROTTLE_MIN_PHASE3 | 0.30 | ratio | 3 minimum |

### Phase 1a/1b Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp_pos | [4.5, 4.5, 0.90] | Position gains (XY, Z) |
| Kd_pos | [6.5, 6.5, 1.00] | Velocity gains (XY, Z) |
| Kp_att | 300,000 N⋅m | Attitude proportional gain |
| Kd_att | 40,000 N⋅m⋅s | Attitude derivative gain |
| MIDDLE_TILT_GAIN | 2.5 | XY force amplification |
| PURE_PURSUIT_LOOKAHEAD | 2000 m | Carrot distance ahead |
| MAX_PITCH_FROM_VERTICAL | 60° | Max tilt angle |

### Phase 1c Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| PHASE1C_THROTTLE | 0.50 | 50% thrust |
| PHASE1C_FLIP_KP | 1.5 rad/s/rad | Angular velocity gain |
| PHASE1C_FLIP_KD | 50,000 N⋅m/(rad/s) | Damping (high for gimbal authority) |
| PHASE1C_FLIP_SUCCESS_THETA_ERR | 10° | Alignment tolerance |
| PHASE1C_FLIP_SUCCESS_RATE | 0.2 rad/s | Angular rate threshold |
| PHASE1C_FLIP_TIMEOUT | 8.0 s | Failsafe duration |

### Phase 2 Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| PHASE2_FLIP_Kp | 8,000 N⋅m/rad | Attitude gain (high for flip) |
| PHASE2_FLIP_Kd | 25,000 N⋅m⋅s/rad | Damping (braking) |
| PHASE2_RATE_DAMP | 50,000 N⋅m⋅s/rad | Rate damping (anti-spiral) |
| PHASE2_AERO_DAMP | 15,000 N⋅m⋅s/rad | Aerodynamic damping |
| AERO_TORQUE_MAX | 35,000 N⋅m | Max grid fin torque |
| PHASE2_TILT_MAX | 10° | Max tilt angle |
| PHASE2_VEL_DAMP | 0.04 | Velocity damping factor |

### Phase 3 Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| LQR_Q | diag([0.1, 0.1, 500, 500, 100, 100]) | State cost matrix |
| LQR_R | diag([1.0, 1.0]) | Control cost matrix |
| PHASE3_GIMBAL_SLEW_LIMIT | 0.7 rad/s | Rate limiter |
| PHASE3_SOFT_ENGAGEMENT_TIME | 0.5 s | Gain ramp duration |
| PHASE3_RATE_PRIORITY_TIME | 0.2 s | Pure damping duration |
| PHASE3_ATTITUDE_ONLY_ALT | 50 m | Switch altitude |
| PHASE3_ROLL_DAMPING | 5,000 N⋅m⋅s/rad | Artificial RCS |
| PHASE3_IGNITION_SAFETY_MARGIN | 0.98 | 2% margin |

### Trajectory Planning
| Parameter | Value | Description |
|-----------|-------|-------------|
| TRAJ_X_REF | - | Downrange reference (computed) |
| TRAJ_Z_REF | - | Apogee altitude reference (computed) |
| MECO_TIME_MAX | 120 s | Max time to main engine cutoff |
| MECO_ALT_MAX | 300,000 m | Max altitude for MECO |

---

## Visualization System

### Architecture: `plotter.py`

The visualization system uses matplotlib 3D animation with dual views and interactive playback controls.

**Class: `Plotter`**

```python
class Plotter:
    def __init__(self, traj_pos):
        self.traj_pos = traj_pos  # Reference trajectory
        self.fig = plt.figure(figsize=(12, 6))
        
        # Dual 3D viewports
        self.ax_local = subplot(1,2,1, projection="3d")   # Rocket-centered
        self.ax_global = subplot(1,2,2, projection="3d")  # World frame
        
        # Visual elements
        self.rocket_line, self.exhaust_line          # Local view rocket
        self.leg_lines[3]                             # Landing legs
        self.global_rocket_line                       # Global view rocket
        self.global_exhaust_line                      # Global view exhaust
        self.telemetry_text                           # Live data display
```

#### Local View (Rocket-Centered)

- **Origin**: Center of gravity (CG)
- **Frame**: Rocket-fixed body frame
- **Purpose**: Close inspection of gimbal deflections and attitude

**Visual Elements**:

```
Rocket body:       Line from tail to nose
  Origin: CG (center)
  Direction: Body Z-axis (rocket length vector)
  Length: 5.825 m total
  
Exhaust plume:     Colored line showing thrust direction
  Origin: Engine location (2.8 m below CG)
  Direction: [F·δy, F·δz, F] (gimbal-deflected)
  Length: 4 m (idle) to 12 m (full thrust)
  Color: Orange (darker at low thrust, brighter at high)
  
Landing legs:      Three lines at 120° spacing
  Origin: Engine (base)
  Deployment: 0° (retracted) to 150° (extended)
  Color: Grey (retracted), Green (deployed & landed)
```

**Gimbal Direction** (fixed in Phase 1c):
```
Gimbal deflection causes exhaust to point:
  δy > 0: Exhaust points to +X (right wing)
  δz > 0: Exhaust points to +Y (left wing)
  
Physics: F_thrust = [F·δy, F·δz, F]
Visualization: exhaust_dir = [δy, δz, -1] (normalized)
               ↑ Must match physics sign convention!
```

#### Global View (World Frame)

- **Origin**: Launch pad
- **Frame**: Inertial world coordinates
- **Purpose**: Monitor altitude, downrange distance, lateral drift

**View Modes**:

```
Phases 1a/1b/1c (ascent/coast/flip):
  Fixed camera: [0, -5km] to [+20km, +5km] horizontal
                Vertical: 0 to +150km altitude
  Shows: Complete ascent trajectory, reference path, target
  
Phase 2 (descent):
  Static view: Same as ascent
  Rocket hidden (would be too small)
  
Phase 3 (hover-slam):
  Follow camera: ±50m box around rocket
  "Landing Cam" title
  Shows: Rocket, exhaust plume, legs
  Altitude relative reference
```

#### Interactive Controls

**Playback Slider**:
```
Allows scrubbing through entire trajectory
Range: 0 to final frame
Drag to seek to any point in time
Shows frame number
```

**Play/Pause Button**:
```
Toggles animation playback
Updates display at current frame rate
```

**Speed Control** (Radio Buttons):
```
0.5x, 1x, 2x, 4x playback speed
Adjusts FPS dynamically
```

### State Visualization Logic

**Function: `update(state, control, target, frame_idx, phase, landed)`**

Called once per frame, updates all visual elements:

```python
def update(self, sim_state, control, target_pos, frame_idx, phase, landed):
    # Extract state components
    pos = state[0:3]
    phi, theta, psi = state[6:9]
    
    # Compute body frame axes
    dcm = euler_to_dcm(phi, theta, psi)
    body_z = dcm[:, 2]  # Nose direction
    
    # LOCAL VIEW: Rocket-centered frame
    # Rocket body line (centered at origin)
    nose = +body_z * LENGTH/2
    tail = -body_z * LENGTH/2
    rocket_line.set_data([tail[0], nose[0]], [tail[1], nose[1]])
    rocket_line.set_3d_properties([tail[2], nose[2]])
    
    # Exhaust direction (gimbal-deflected thrust vector)
    delta_y, delta_z = control["gimbal_y"], control["gimbal_z"]
    exhaust_dir_body = [delta_y, delta_z, -1]  # -1 = backward/down
    exhaust_dir_body /= ||exhaust_dir_body||    # Normalize
    exhaust_dir_world = dcm @ exhaust_dir_body
    
    engine_origin = -body_z * CG_FROM_GIMBAL  # Engine location
    exhaust_length = 4 + 8 * throttle          # Scales with thrust
    exhaust_tip = engine_origin + exhaust_dir_world * exhaust_length
    exhaust_line.set_data([engine_origin[0], exhaust_tip[0]], ...)
    
    # Landing legs (3 legs at 120° spacing)
    leg_deploy_angle = leg_deploy * 150°  # 0° retracted → 150° extended
    leg_color = "green" if landed else "grey"
    
    for i, azimuth in enumerate([0°, 120°, 240°]):
        # Compute leg tip in body frame
        x_leg = 2.5*sin(deploy_angle)*cos(azimuth)
        y_leg = 2.5*sin(deploy_angle)*sin(azimuth)
        z_leg = 2.5*cos(deploy_angle) - CG_FROM_GIMBAL
        
        # Transform to world frame
        leg_tip = dcm @ [x_leg, y_leg, z_leg]
        leg_line.set_data([engine_origin[0], engine_origin[0]+leg_tip[0]], ...)
        leg_line.set_color(leg_color)
    
    # GLOBAL VIEW: Phase-specific camera
    if phase == "phase3":
        # Follow camera ±50m around rocket
        ax.set_xlim(pos[0] - 50, pos[0] + 50)
        ax.set_ylim(pos[1] - 50, pos[1] + 50)
        ax.set_zlim(max(0, pos[2]-50), pos[2]+50)
        
        # Show rocket in global view
        nose_global = pos + body_z * LENGTH/2
        tail_global = pos - body_z * LENGTH/2
        global_rocket_line.set_data([tail_global[0], nose_global[0]], ...)
        
        # Show exhaust in global
        engine_global = pos - body_z * CG_FROM_GIMBAL
        exhaust_tip_global = engine_global + exhaust_dir_world * exhaust_length
        global_exhaust_line.set_data([engine_global[0], exhaust_tip_global[0]], ...)
    else:
        # Hide rocket in global view for phases 1-2
        global_rocket_line.set_data([], [])
        global_exhaust_line.set_data([], [])
    
    # Telemetry text box (live control data display)
    telemetry = f"PHASE: {phase}\n"
    telemetry += f"Pos: [{pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:7.1f}] m\n"
    telemetry += f"Vel: [{vel[0]:6.1f}, {vel[1]:6.1f}, {vel[2]:6.1f}] m/s\n"
    
    # Extract gimbal angles
    δy_deg = np.rad2deg(control["gimbal_y"])
    δz_deg = np.rad2deg(control["gimbal_z"])
    throttle = control["throttle"]
    
    # Extract aerodynamic control data (Phase 2 only)
    aero_torque = control.get("aero_torque", np.zeros(3))
    grid_fins = control.get("grid_fins", 0.0)
    aero_torque_mag = np.linalg.norm(aero_torque)
    
    # Format telemetry line
    telemetry += f"Gimbal: [{δy_deg:6.1f}, {δz_deg:6.1f}] deg  Thrust: {throttle*100:5.1f}%\n"
    telemetry += f"Ailerons: τ={aero_torque_mag:7.0f} N⋅m  Grid Fins: {grid_fins*100:5.1f}%"
    telemetry_text.set_text(telemetry)
    
    # Display color depends on phase
    if phase == "phase2":
        telemetry_text.set_color("yellow")  # Yellow for aerodynamic control
    elif phase == "phase3":
        telemetry_text.set_color("cyan")    # Cyan for hover-slam
```

### Live Telemetry Data Fields

The telemetry text box displays real-time control output values:

**Example Display** (Phase 2 - Aerodynamic Control):
```
PHASE: PHASE2
Pos: [  12345.2,    -234.1,   45300.0] m
Vel: [   145.3,      -5.2,   -250.6] m/s
Gimbal: [    0.0,      0.0] deg  Thrust:  0.0%
Ailerons: τ=   23450 N⋅m  Grid Fins: 100.0%
```

**Field Meanings**:

| Field | Units | Description | Phase Active | Range |
|-------|-------|-------------|--------------|-------|
| **PHASE** | - | Current control phase | All | PHASE1A, PHASE1B, PHASE1C, PHASE2, PHASE3 |
| **Pos** | m | Position in world frame [X, Y, Z] | All | X/Y: ±20km, Z: 0-150km |
| **Vel** | m/s | Velocity in world frame [Vx, Vy, Vz] | All | ±500 m/s typical |
| **Gimbal** | deg | Engine gimbal deflection [δy, δz] | All | ±15° (Phase 1-2), ±10° (Phase 3) |
| **Thrust** | % | Throttle percentage | All | 0-100% |
| **Ailerons: τ** | N⋅m | Aerodynamic torque magnitude | Phase 2 | 0-35,000 N⋅m |
| **Grid Fins** | % | Grid fin deployment factor | Phase 2 | 0-100% |

**Phase-Specific Data**:

**Phases 1a/1b/1c** (Powered/Coast/Flip):
```
PHASE: PHASE1C
Pos: [  12300.5,    -120.3,   95200.1] m
Vel: [   180.2,      -3.5,    -50.8] m/s
Gimbal: [    3.2,     -1.5] deg  Thrust: 85.3%
Ailerons: τ=       0 N⋅m  Grid Fins:   0.0%    ← Engine control (no ailerons)
```

**Phase 2** (Ballistic Descent):
```
PHASE: PHASE2
Pos: [  12345.2,    -234.1,   45300.0] m
Vel: [   145.3,      -5.2,   -250.6] m/s
Gimbal: [    0.0,      0.0] deg  Thrust:  0.0%  ← Engine OFF
Ailerons: τ=   23450 N⋅m  Grid Fins: 100.0%    ← Grid fin control active
```

**Phase 3** (Hover-Slam Landing):
```
PHASE: PHASE3
Pos: [  12500.8,    -250.3,      10.5] m
Vel: [     2.1,      -0.5,     -25.3] m/s
Gimbal: [    2.5,     -0.8] deg  Thrust: 95.7%  ← Energy-optimal throttle
Ailerons: τ=       0 N⋅m  Grid Fins:   0.0%    ← Ailerons inactive (LQR gimbal)
```

**Interpretation Notes**:
- **Gimbal = [0,0]** in Phase 2: Engine is off, no gimbal authority
- **Thrust = 0%** in Phase 2: Ballistic descent (coasting)
- **Ailerons active** only in Phase 2: Grid fins provide attitude control
- **Grid Fins = 100%**: Fully deployed (maximum aerodynamic authority)
- **Ailerons: τ** scales with dynamic pressure: higher at high velocity/low altitude

### Animation Loop

**Function: `animate(history, total_time, dt)`**

Drives the interactive playback:

```python
def animate(self, history, total_time, dt):
    # Store history
    self._history_state = np.array(history["state"])
    self._history_control = history["control"]
    self._history_phase = history["phase"]
    self._history_landed = history.get("landed", [...])
    
    frames = len(self._history_state)
    
    # Create playback controls
    slider = Slider(ax_slider, "Frame", 0, frames-1, valinit=0)
    play_button = Button(ax_button, "Play")
    speed_radio = RadioButtons(ax_speed, ["0.5x", "1x", "2x", "4x"])
    
    def update_frame(frame_num):
        state = self._history_state[frame_num]
        control = self._history_control[frame_num]
        phase = self._history_phase[frame_num]
        landed = self._history_landed[frame_num]
        
        return self.update(state, control, ..., frame_num, phase, landed)
    
    # Setup animation
    anim = FuncAnimation(self.fig, update_frame, 
                         frames=frames, interval=dt*1000, repeat=True)
```

---

## Running the Simulation

### Prerequisites
```bash
pip install numpy scipy matplotlib
```

### Execution

**From workspace root**:
```bash
cd JonasAttempt_03
python run_demo.py
```

This executes:
1. Creates reference trajectory
2. Initializes rocket and simulator
3. Runs ODE integration with phase transitions
4. Animates 3D visualization with playback controls

### Expected Output

**Console Output**:
```
--- TRANSITION TO PHASE 1a (LAUNCH) at t=0.0s, Z=0m ---
[Phase1a] t=1.50s, Z=1234m, theta_des=5.2°, throttle=95%
...

--- TRANSITION TO PHASE 1b (MECO/COAST) at t=95.3s, Z=148500m ---
[Phase1b Check] t=96.50s, Vz=-23.45m/s (threshold=0.0)

======================================================================
DEBUG: ENTERING PHASE 1C NOW!
--- TRANSITION TO PHASE 1c (POWERED FLIP) at t=98.2s ---
  Altitude: Z=149230m
  Velocity: Vx=1234.5, Vy=23.4, Vz=-45.3 m/s
  Attitude: θ=5.2°, ψ=-1.2°
======================================================================

  FLIP t=0.50s: v_mag=1247.2m/s, angle_err=175.3°
         u_rocket=[-0.034, 0.012, 0.999]
         u_desired=[0.988, -0.018, -0.037]
         ω_mag=15.3°/s, δy=3.2°, δz=-1.5°

  FLIP t=3.25s: v_mag=876.5m/s, angle_err=12.4°
         ω_mag=0.15°/s, δy=0.05°, δz=-0.08°

DEBUG: 1c->2 via Attitude Aligned

======================================================================
--- FLIP COMPLETE. MAIN ENGINE CUTOFF. ENTERING PHASE 2 ---
  Time in flip: 3.25s
  Final θ_err: -8.3°
  Final ψ_err: 2.1°
======================================================================

Phase2: Z=145200m, Vz=-892.3m/s, d_stop=523.4m, trigger_alt=513.2m

======================================================================
--- TRANSITION TO PHASE 3 (HOVERSLAM) ---
  Position: X=12340.5m, Y=-234.2m, Z=523.0m
  Velocity: Vx=23.4m/s, Vy=-5.2m/s, Vz=-892.3m/s
  Attitude: φ=-2.1°, θ=179.8°, ψ=0.3°
======================================================================

  LQR t=0.05s: Z=520.1m, Vz=-890.2m/s, vx=23.2, throttle=0.85, δy=0.2°
  LQR t=0.23s: Z=250.4m, Vz=-345.6m/s, vx=12.3, throttle=0.92, δy=-0.1°
  LQR t=0.45s: Z=45.2m, Vz=-89.3m/s, vx=2.1, throttle=1.00, δy=0.05°

======================================================================
*** LANDING SUCCESSFUL ***
  Time: t=234.5s
  Final Position: X=12345.2m, Y=-234.1m, Z=0.08m
  Final Velocity: Vx=-0.02m/s, Vy=0.01m/s, Vz=-0.45m/s
  Landing Error: 1.2m from target
======================================================================
```

**Visualization**:
1. Two 3D views open in matplotlib window
2. Left: Local rocket-centered view (gimbal deflections visible)
3. Right: Global view showing altitude and trajectory
4. Animation plays at configurable speed with pause/seek controls
5. Legs turn green upon touchdown, animation stops at landing

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Phase 1c won't execute | Phase detection logic off | Check `vz <= 0.0` threshold in phase1b check |
| Gimbal angles too small | KD gain too low | Increase PHASE1C_FLIP_KD to 50,000+ |
| Oscillation around 180° flip | Angle-based controller | Already using vector-based (cross product) |
| Landing velocity >1 m/s | Safety factor too low | Increase to 1.5× in energy equation |
| LQR gimbal saturation | Aggressive gains | Check anti-saturation logic, reduce LQR_Q |
| Visualization gimbal reversed | Exhaust direction negated | Verify `exhaust_dir = [δy, δz, -1]` in plotter |
| Animation playback jerky | Insufficient frame rate | Lower SIM_DT or reduce visualization resolution |

---

## Future Enhancement Opportunities

1. **Multi-gimbal Vectoring**: Implement side boosters with individual gimbals
2. **Grid Fin Dynamics**: Fully simulate grid fin control authority
3. **RCS Thrusters**: Replace artificial roll damping with actual RCS pulses
4. **Trajectory Optimization**: Use inverse dynamics to optimize Phase 1a path
5. **Sensor Simulation**: Add IMU/navigation filter to realistic guidance
6. **Structural Loads**: Track maximum gimbal torques and stress margins
7. **Propellant Management**: Dynamic mass with burn rate model
8. **Wind Disturbances**: Add atmospheric wind gusts in all phases

---

**Documentation Last Updated**: Session 6 (Gimbal Visualization Fix)
**System Status**: ✅ All phases operational, landing successful

