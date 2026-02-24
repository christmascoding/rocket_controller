# Complete Parameter Reference

This document lists all configuration parameters with their purposes and effects.

---

## Physical Constants

```python
G = 9.81                # Gravitational acceleration (m/s²)
RHO = 1.225             # Air density at sea level (kg/m³)
SIM_DT = 0.05           # Simulation timestep (seconds) - RK45 integration
```

---

## Rocket Geometry & Mass Properties

### Dimensions
```python
LENGTH = 5.825          # Overall rocket length (m)
DIAMETER = 0.7          # Body diameter (m)
RADIUS = 0.35           # Body radius = DIAMETER/2 (m)
```

### Mass
```python
MASS = 10_000.0         # Rocket wet mass (kg) - constant throughout simulation
                        # Note: Real rockets would have mass_dot and decreasing mass
```

### Thrust
```python
MAX_THRUST = 300_000.0  # Maximum engine thrust (N)
                        # At sea level, vacuum would be higher
```

### Center of Gravity & Gimbal
```python
CG_FROM_GIMBAL = 2.8    # Distance from engine gimbal to CG (m)
                        # Positive = CG above gimbal (nose-up configuration)
                        # Creates lever arm for gimbal torque: τ = r × F
```

### Aerodynamic Reference Area
```python
S_REF = π·RADIUS² ≈ 0.385  # Body cross-sectional area (m²)
                             # Used for drag force: F_drag = ½·ρ·v²·S·CD
```

---

## Moments of Inertia

Computed using **cylinder approximation** (uniform mass distribution):

```python
I_ZZ = 0.5 * MASS * RADIUS²
     = 0.5 * 10,000 * 0.35²
     ≈ 612.5 kg⋅m²        # Roll inertia (rotation about nose axis)

I_XX = (1/12) * MASS * (3*RADIUS² + LENGTH²)
     = (1/12) * 10,000 * (3*0.35² + 5.825²)
     ≈ 28,971 kg⋅m²       # Pitch inertia (rotation about starboard axis)

I_YY = I_XX ≈ 28,971 kg⋅m²  # Yaw inertia (rotation about port axis)
                             # Equal to I_XX by symmetry

INERTIA = diag([I_XX, I_YY, I_ZZ])  # 3×3 inertia tensor
INERTIA_INV = inv(INERTIA)           # Pre-computed for efficiency
```

**Physical Meaning**:
- High I_XX, I_YY: Rocket is long and thin (hard to pitch/yaw)
- Lower I_ZZ: Easy to roll (small cross-sectional moment arm)

---

## Aerodynamics

```python
CL_ALPHA = 3.5          # Lift coefficient slope (1/radian)
                        # dCL/dα = 3.5 ⟹ sharp, stable airfoil
                        # Used: lift_coeff = CL_ALPHA * α

CD0 = 0.3               # Zero-lift drag coefficient (parasitic drag)
                        # At zero angle of attack

CD_ALPHA = 1.2          # Drag due to angle of attack (quadratic term)
                        # CD = CD0 + CD_ALPHA * α²
```

### Center of Pressure Offset

```python
CP_OFFSET_ASCENT = 0.6  # m, behind CG during ascent (Phase 1a/1b)
                        # Creates stabilizing moment during climb
                        # "Behind" means toward engine (-Z direction in body frame)

CP_OFFSET_PHASE2 = 0.8  # m, ahead of CG during descent (Phase 2)
                        # Grid fins deployed, shift CP forward
                        # Provides control torque for attitude management
```

**Effect on Control Authority**:
```
Aerodynamic moment = CP_offset × drag_force
Larger offset → More control authority from aerodynamics
Phase 2: Uses high CP offset for grid fin control
```

---

## Control Limits

### Gimbal Angle Limits

```python
GIMBAL_MAX_DEG = 15.0          # Nominal gimbal range (degrees)
GIMBAL_MAX = np.deg2rad(15.0)  # ≈ 0.2618 radians

GIMBAL_MAX_PHASE3_DEG = 10.0   # Reduced during landing (Phase 3)
GIMBAL_MAX_PHASE3 = np.deg2rad(10.0)
                               # Smaller angles → more vertical thrust
                               # Less tilt authority, more hover capability

LAUNCH_GIMBAL_MAX_DEG = 3.0    # Pure pursuit phase (Phase 1a) only
LAUNCH_GIMBAL_MAX = np.deg2rad(3.0)
                               # Very conservative during ascent
                               # Prevents large lateral accelerations
```

**Why Different Limits?**
- **Ascent (1a)**: Small gimbal keeps trajectory aligned (prevent drift)
- **Descent (2)**: Large gimbal for attitude control via aero (no thrust)
- **Landing (3)**: Reduced gimbal to maximize vertical component of thrust

### Throttle Limits

```python
THROTTLE_MIN_PHASE3 = 0.30     # Minimum throttle in hover-slam (30%)
                               # Below 30%, not enough thrust to hover
                               # Used in phase3_controller:
                               #   throttle = clamp(..., 0.30, 1.0)
```

### Aerodynamic Torque Authority

```python
AERO_TORQUE_MAX = 35_000.0     # Maximum torque from grid fins (N⋅m)
                               # Used in Phase 2 to limit aero surface commands
                               # Prevents unrealistic control authority
```

---

## Phase 1a/1b: Ascent & Coast

### Guidance (Pure Pursuit)

```python
PURE_PURSUIT_LOOKAHEAD = 2000.0  # Look-ahead distance (m)
                                 # Carrot point at 2km ahead on trajectory
                                 # Prevents overshooting/zigzagging

MAX_PITCH_FROM_VERTICAL_DEG = 60.0  # Maximum pitch angle (degrees)
MAX_PITCH_FROM_VERTICAL = np.deg2rad(60.0)
                                     # Limits how far off-vertical rocket can tilt
                                     # 60° tilt = significant horizontal acceleration

PURE_PURSUIT_END_THRESHOLD = 0.95   # Disable PP at 95% of path
                                    # Final 5% → hold current attitude
                                    # Prevents control saturation at end
```

### Outer Loop (Position Error → Acceleration)

```python
Kp_pos = np.array([4.5, 4.5, 0.90])  # Position proportional gains
                                      # XY: 4.5 (aggressive horizontal tracking)
                                      # Z: 0.90 (gentler vertical control)

Kd_pos = np.array([6.5, 6.5, 1.00])  # Velocity proportional gains (damping)
                                      # XY: 6.5 (damp horizontal velocities)
                                      # Z: 1.00 (damp vertical velocities)
```

**Physics**:
```
a_cmd = Kp_pos * (pos_error) + Kd_pos * (vel_error)
F_desired = mass * (a_cmd + [0, 0, g])

Example at apogee with 1000m altitude error:
  a_cmd_z = 0.90 * (-1000) + 1.00 * (vz_error) ≈ -900 m/s²  (huge acceleration!)
  F_z = 10,000 * (-900 + 9.81) = -8,900,190 N  (over 890× gravity!)
  → Clipped to max available thrust
```

### Inner Loop (Attitude Error → Gimbal)

```python
Kp_att = 300_000.0  # Attitude proportional gain (N⋅m/radian)
                    # τ = 300,000 * (u_body × u_desired)
                    # At 1 radian (57°) error: τ = 300,000 N⋅m

Kd_att = 40_000.0   # Attitude derivative gain (N⋅m⋅s/radian)
                    # τ -= 40,000 * ω
                    # Damps angular rates strongly
```

**Gimbal Conversion**:
```
δ_gimbal = τ / (CG_FROM_GIMBAL * F)
         = τ / (2.8 * 285,000)  [at 95% throttle]
         = τ / 798,000

At max torque τ = 300,000 N⋅m:
  δ_gimbal = 300,000 / 798,000 ≈ 0.375 rad ≈ 21.5°
  Saturates at 15° gimbal limit → actual τ ≈ 119,700 N⋅m
```

### Middle Loop (Force → Attitude & Throttle)

```python
MIDDLE_TILT_GAIN = 2.5  # Amplifies lateral force command
                        # Used in middle_loop() to emphasize XY components:
                        # F_adjusted = [F_x * 2.5, F_y * 2.5, F_z]
                        # Makes guidance more aggressive in horizontal plane
```

### Phase Transition Thresholds

```python
MECO_TIME_MAX = 120.0       # Maximum time in Phase 1a (seconds)
                            # Failsafe: even if not at target, cutoff at 120s

MECO_ALT_MAX = 300_000.0    # Maximum altitude for MECO (m)
                            # Failsafe: cut off if altitude exceeds 300 km

TRAJ_X_REF = ???            # Computed from trajectory planning
TRAJ_Z_REF = ???            # Computed from trajectory planning
                            # When x ≥ TRAJ_X_REF OR z ≥ TRAJ_Z_REF → Phase 1b
```

---

## Phase 1c: Powered Apogee Flip

### Throttle & Gimbal Control

```python
PHASE1C_THROTTLE = 0.50  # 50% engine thrust
                         # Gimbal is primary actuator
                         # F = 0.50 * 300,000 = 150,000 N
                         # Provides lever arm: τ = F * L = 420,000 N⋅m max

PHASE1C_FLIP_KP = 1.5    # Proportional gain (rad/s per radian error)
                         # ω_desired = rotation_axis * (1.5 * angle_error)
                         # Example: 90° error → 1.5 * π/2 ≈ 2.36 rad/s desired spin

PHASE1C_FLIP_KD = 50000.0  # Derivative gain (N⋅m per rad/s error)
                            # τ = 50,000 * (ω_desired - ω_current)
                            # High value ensures gimbal saturates when needed
```

**Why High KD?**
```
Goal: Convert ω_error into gimbal angle
  δ = τ / (F * L) = (KD * ω_error) / (F * L)

At 150kN thrust, L=2.8m:
  δ = 50,000 * ω_error / 420,000
    = 0.119 * ω_error (radians per rad/s)
    
At large ω_error (e.g., 10 rad/s):
  δ = 1.19 rad ≈ 68° ← Saturates at 15°
  Actual: τ = 420,000 * 0.26 = 109,200 N⋅m
  Effective ω_error = 109,200 / 50,000 ≈ 2.2 rad/s
  ω_actual_acceleration = τ / I_yy = 109,200 / 28,971 ≈ 3.8 rad/s²
```

### Success Criteria

```python
PHASE1C_FLIP_SUCCESS_THETA_ERR = np.deg2rad(10.0)  # 10° pitch alignment
PHASE1C_FLIP_SUCCESS_RATE = 0.2                     # 0.2 rad/s = 11.5°/s max
                                                    # Rate threshold for stabilization

# Must satisfy all:
#   1. time_in_phase1c > 0.5 s (minimum dwell time)
#   2. |theta_error| < 10°
#   3. |psi_error| < 10°
#   4. angular_rate < 0.2 rad/s
#   OR
#   5. time_in_phase1c > 8.0 s (failsafe timeout)
```

### Failsafe

```python
PHASE1C_FLIP_TIMEOUT = 8.0  # Transition to Phase 2 after 8 seconds
                            # Prevents infinite flip loops
                            # Typical flip: 3-5 seconds
                            # Margin: 3-5 seconds extra buffer
```

---

## Phase 2: Ballistic Descent

### Attitude Control (Grid Fins / Aero Surfaces)

```python
PHASE2_FLIP_Kp = 8_000.0      # Attitude proportional gain (N⋅m/rad)
PHASE2_FLIP_Kd = 25_000.0     # Attitude derivative gain (N⋅m⋅s/rad)
                              # High Kd for braking effect during flip

PHASE2_RATE_DAMP = 50_000.0   # Rate damping gain (N⋅m⋅s/rad)
                              # Pure angular rate damping
                              # Prevents spiral divergence

PHASE2_AERO_DAMP = 15_000.0   # Aerodynamic damping gain (N⋅m⋅s/rad)
                              # Simulates natural aero damping
```

### Attitude Limits

```python
PHASE2_TILT_MAX_DEG = 10.0    # Maximum tilt during descent (degrees)
PHASE2_TILT_MAX = np.deg2rad(10.0)
                             # Prevents excessive horizontal velocity growth

PHASE2_VEL_DAMP = 0.04       # Velocity damping factor
                             # Reduces horizontal velocity over time
```

### Ignition Trigger (Phase 2 → 3 Transition)

```python
PHASE3_IGNITION_SAFETY_MARGIN = 0.98  # Dimensionless, [0, 1]
                                       # Ignite at 98% of calculated burn distance
                                       # Provides 2% safety margin for precision burn

# Trigger logic in simulator._should_ignite():
amax = F_max / mass - g              # Maximum deceleration available
dstop = vz² / (2 * amax)            # Distance to stop from current velocity
trigger_altitude = Z * MARGIN       # Ignite when Z ≤ trigger_altitude
should_ignite = (vz < 0) AND (dstop ≥ trigger_altitude)

# Example:
# Z = 100m, Vz = -300 m/s
# amax = (300,000 / 10,000) - 9.81 = 20.19 m/s²
# dstop = 300² / (2 * 20.19) = 2,228 m  (too high!)
# 
# Z = 2228m, Vz = -300 m/s, amax same
# dstop = 2,228m
# trigger_alt = 2,228 * 0.98 = 2,184m
# should_ignite = YES! (2,228 ≥ 2,184)
```

---

## Phase 3: Hover-Slam (Energy-Optimal Descent with LQR)

### Vertical Throttle Control

```python
# Energy-optimal formula:
a_required = (vz² / (2 * z)) * 1.5  # 50% safety margin

# Physics: v² = 2*a*d  ⟹  a = v² / (2*d)
# This deceleration brings velocity to zero exactly at ground
# Safety factor 1.5 increases deceleration for margin

# Throttle computation:
thrust_required = mass * (a_required + g)
throttle = clamp(thrust_required / F_max, 0.30, 1.0)
```

### LQR State Feedback

```python
LQR_Q = np.diag([0.1, 0.1, 500, 500, 100, 100])
#                  ↑    ↑    ↑    ↑    ↑    ↑
#                  vx   vy   θ_e  ψ_e  q    r
#
# Cost for state vector: [vx, vy, θ_err, ψ_err, q, r]
# Weights (higher = more penalized):
#   Velocities (0.1): Minor cost, allows drift
#   Attitude errors (500): Highest cost, maintain upright
#   Angular rates (100): Medium cost, dampen rotation

LQR_R = np.diag([1.0, 1.0])  # Control cost: [δy, δz]
                             # Equal weight on pitch and yaw gimbal
                             # 1.0 is baseline; higher = less aggressive control
```

**Interpretation**:
```
Total cost = x.T @ Q @ x + u.T @ R @ u

High Q on attitude: "Stay upright at all costs"
Low Q on velocity: "Drift is OK, focus on attitude"
Moderate Q on rates: "Damp oscillations but don't overcontrol"

LQR minimizes this cost, produces optimal feedback gain:
  K = R⁻¹ @ B.T @ P  (from Riccati solution)
  u_optimal = -K @ x
```

### Gimbal Rate Limiting (Saturation Prevention)

```python
PHASE3_GIMBAL_SLEW_LIMIT = 0.7  # rad/s
                                # Maximum gimbal rate of change
                                # Prevents bang-bang controller behavior

# Applied as:
max_delta = SLEW_LIMIT * dt
gimbal_limited = clamp(gimbal_command, 
                       prev_gimbal - max_delta,
                       prev_gimbal + max_delta)

# At dt=0.05s:
max_delta = 0.7 * 0.05 = 0.035 rad ≈ 2°/frame
# Smooth transition over multiple frames
```

### Soft Engagement (Gain Ramp-Up)

```python
PHASE3_RATE_PRIORITY_TIME = 0.2  # seconds
# First 0.2s: Pure rate damping only
#   x = [0, 0, 0, 0, q, r]
#   Purpose: Stabilize any residual spin from Phase 1c

PHASE3_SOFT_ENGAGEMENT_TIME = 0.5  # seconds
# Total ramp-up time from 0.0 to 0.5 seconds
# 0.0-0.2s: Rate damping only
# 0.2-0.5s: Blend in position/attitude (linear ramp)
# 0.5s+: Full state feedback

# Blend calculation:
if time < RATE_PRIORITY_TIME:
    blend = 0.0  # Pure damping
elif time < SOFT_ENGAGEMENT_TIME:
    blend = (time - RATE_PRIORITY_TIME) / (SOFT_ENGAGEMENT_TIME - RATE_PRIORITY_TIME)
    blend ∈ [0, 1]  # Gradual transition
else:
    blend = 1.0  # Full feedback
    
x = blend * [vx, vy, theta_err, psi_err, q, r] + [0, 0, 0, 0, q, r]
```

### Altitude-Based Mode Switching

```python
PHASE3_ATTITUDE_ONLY_ALT = 50.0  # meters

# Above 50m: Attitude-only mode
#   x = [0, 0, θ_err, ψ_err, q, r]
#   Ignores horizontal velocity drift
#   Maximizes vertical thrust component

# Below 50m: Full state feedback
#   x = [vx, vy, θ_err, ψ_err, q, r]
#   Corrects horizontal position
#   Final precision landing

# Why? At high altitude, vertical thrust dominates
# Tilting for horizontal correction "wastes" vertical component
# Near ground, can afford tilt for horizontal precision
```

### Roll Damping (RCS Simulation)

```python
PHASE3_ROLL_DAMPING = 5_000.0  # N⋅m⋅s/rad
                               # Artificial roll damping term
                               # In real rocket: reaction control system (RCS)

# Applied in dynamics:
m_roll_damp = -ROLL_DAMPING * p  # Torque opposes roll rate
# Example: p = 0.1 rad/s
#   m = -5000 * 0.1 = -500 N⋅m
#   ω̇_p = -500 / I_xx = -500 / 28,971 ≈ -0.017 rad/s²
```

---

## Trajectory Planning

```python
TRAJ_X_REF = ???  # Downrange reference distance (computed)
TRAJ_Z_REF = ???  # Apogee altitude reference (computed)

# Computed from trajectory planning algorithm
# Used to define Phase 1a → 1b transition
```

---

## Launch Phase Limits

```python
LAUNCH_T_LIMIT = 20.0           # Max time in Phase 1a launch (seconds)
LAUNCH_POS_ERR_MAX = 500.0      # Max position error during launch (m)
LAUNCH_VEL_ERR_MAX = 50.0       # Max velocity error during launch (m/s)

# Prevent extreme corrections during ascent
# Clamp outer loop commands to avoid unrealistic accelerations
```

---

## Summary Table

| Category | Parameter | Value | Unit | Purpose |
|----------|-----------|-------|------|---------|
| **Physical** | G | 9.81 | m/s² | Gravity |
| | RHO | 1.225 | kg/m³ | Air density |
| **Rocket** | LENGTH | 5.825 | m | Body length |
| | MASS | 10,000 | kg | Wet mass |
| | MAX_THRUST | 300,000 | N | Max engine thrust |
| | CG_FROM_GIMBAL | 2.8 | m | Gimbal-CG distance |
| **Inertia** | I_XX, I_YY | 28,971 | kg⋅m² | Pitch/yaw inertia |
| | I_ZZ | 612.5 | kg⋅m² | Roll inertia |
| **Aero** | CL_ALPHA | 3.5 | 1/rad | Lift slope |
| | CD0 | 0.3 | - | Parasitic drag |
| | CP_OFFSET_ASCENT | 0.6 | m | CP location (phase 1) |
| **Gimbal** | GIMBAL_MAX | 15 | degrees | Nominal limit |
| | GIMBAL_MAX_PHASE3 | 10 | degrees | Landing limit |
| **Phase 1a/b** | PURE_PURSUIT_LOOKAHEAD | 2000 | m | Guidance distance |
| | Kp_pos (XY) | 4.5 | - | Position gain |
| | Kd_pos (XY) | 6.5 | - | Velocity gain |
| | Kp_att | 300,000 | N⋅m/rad | Attitude gain |
| | Kd_att | 40,000 | N⋅m⋅s/rad | Rate gain |
| **Phase 1c** | PHASE1C_THROTTLE | 0.50 | ratio | Engine setting |
| | PHASE1C_FLIP_KP | 1.5 | rad/s/rad | Proportional gain |
| | PHASE1C_FLIP_KD | 50,000 | N⋅m⋅s/rad | Derivative gain |
| | PHASE1C_FLIP_SUCCESS_THETA_ERR | 10 | degrees | Alignment tolerance |
| | PHASE1C_FLIP_TIMEOUT | 8.0 | seconds | Failsafe duration |
| **Phase 2** | PHASE2_FLIP_Kp | 8,000 | N⋅m/rad | Attitude gain |
| | PHASE2_FLIP_Kd | 25,000 | N⋅m⋅s/rad | Damping gain |
| | PHASE2_RATE_DAMP | 50,000 | N⋅m⋅s/rad | Rate damping |
| | AERO_TORQUE_MAX | 35,000 | N⋅m | Aero authority |
| **Phase 3** | LQR_Q diagonal | [0.1, 0.1, 500, 500, 100, 100] | - | State weights |
| | LQR_R diagonal | [1.0, 1.0] | - | Control weights |
| | PHASE3_GIMBAL_SLEW_LIMIT | 0.7 | rad/s | Rate limiter |
| | PHASE3_SOFT_ENGAGEMENT_TIME | 0.5 | s | Gain ramp-up |
| | PHASE3_ATTITUDE_ONLY_ALT | 50 | m | Mode switch altitude |
| | PHASE3_ROLL_DAMPING | 5,000 | N⋅m⋅s/rad | Roll control |
| | PHASE3_IGNITION_SAFETY_MARGIN | 0.98 | ratio | Burn margin |
| | THROTTLE_MIN_PHASE3 | 0.30 | ratio | Min throttle |

---

**Complete as of Session 6 - All Parameters Documented**

