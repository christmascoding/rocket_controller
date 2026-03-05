# Quick Reference & Developer Guide

## Quick Start

```bash
cd JonasAttempt_03
python run_demo.py
```

Simulation runs automatically, then displays 3D animation with playback controls.

---

## Key Files & Their Roles

| File | Lines | Purpose | Key Classes/Functions |
|------|-------|---------|----------------------|
| **config.py** | 117 | All parameters | Constants only, no functions |
| **utils.py** | 59 | Math utilities | `normalize_angle`, `euler_to_dcm`, `body_z_axis_in_world` |
| **models/rocket.py** | 22 | Rocket properties | `Rocket` class, gimbal/CG geometry |
| **models/trajectory.py** | - | Pre-planned path | `sample_trajectory()` |
| **physics/dynamics.py** | 70 | State derivative | `state_derivative(t, state, control, rocket, phase)` |
| **physics/aerodynamics.py** | - | Aero forces/torques | `compute_aero_forces_moments()` |
| **controllers/cascaded.py** | 410 | All controllers | `pure_pursuit()`, `outer_loop()`, `middle_loop()`, `inner_loop()`, `phase1c_controller()`, `phase2_flip_controller()`, `phase3_controller()`, `lqr_gain()` |
| **simulation/simulator.py** | 328 | Main ODE integrator | `Simulator` class, phase transitions, state machine |
| **visualization/plotter.py** | 323 | 3D animation | `Plotter` class, update/animate methods |

---

## Core Simulation Loop

```python
# In simulator.py, main execution loop:

while t < t_final:
    # 1. Determine current phase
    phase = _update_phase(t)  # phase1a → phase1b → phase1c → phase2 → phase3
    
    # 2. Compute control commands
    control = compute_control(t)  # Based on phase
    
    # 3. Integrate state forward
    state_dot = state_derivative(t, state, control, rocket, phase)
    state = RK45_integrate(state_dot, dt=0.05)
    
    # 4. Store history for visualization
    history.append({state, control, phase, ...})
    
    # 5. Update visualization
    plotter.update(state, control, ...)
    
    t += dt
```

---

## Phase Transition Logic

```
PHASE 1a (Ascent)
  Entry:  t=0, Z=0
  Control: Pure Pursuit guidance → outer/middle/inner loop
  Throttle: 95% (constant)
  Exit condition: t > 120s OR x > x_ref OR z > 150km
  
  ↓ MECO (Main Engine Cutoff)
  
PHASE 1b (Coast)
  Entry:  At MECO altitude/time
  Control: Same guidance loops, throttle=0 (coasting)
  Exit condition: Vz ≤ 0 (apogee reached)
  
  ↓ APOGEE DETECTED
  
PHASE 1c (Powered Flip)
  Entry:  At apogee when Vz=0
  Control: Vector-based gimbal PD control
  Throttle: 50% (gimbal is primary)
  Exit condition: Attitude aligned + stabilized (0.5s) OR timeout (8s)
  
  ↓ FLIP COMPLETE
  
PHASE 2 (Descent)
  Entry:  After Phase 1c success
  Control: Aerodynamic attitude control (nose-down)
  Throttle: 0% (ballistic)
  Exit condition: Energy-optimal burn condition (Z near d_stop)
  
  ↓ IGNITION
  
PHASE 3 (Hover-Slam)
  Entry:  When Z ≤ d_stop
  Control: LQR attitude + energy-optimal throttle
  Throttle: 30-100% (variable)
  Exit condition: Z < 0.5m AND Vz > -1 m/s (landed)
  
  ↓ TOUCHDOWN
  
LANDED (Final)
  Entry:  Upon ground contact
  Control: None (frozen state)
  Animation: Stops at final frame
```

---

## Modifying Parameters

### Most Common Tuning Parameters

**Landing Precision** (altitude and velocity):
```python
# config.py
PHASE3_IGNITION_SAFETY_MARGIN = 0.98  # 0.98 = 2% margin
                                       # Higher = earlier ignition → lower landing alt
                                       # Example: 0.95 ignites earlier, lands ~0.05m lower

# How it works: ignition_altitude = Z × margin
# At Z=100m: margin=0.98 → ignite at 98m
# At Z=100m: margin=0.95 → ignite at 95m
```

**Phase 1c Gimbal Authority**:
```python
PHASE1C_THROTTLE = 0.50              # Higher → more gimbal torque
PHASE1C_FLIP_KD = 50000.0            # Higher → stronger gimbal response
                                      # At 50% thrust: τ = 50000 × ω_error
                                      # τ/F/L = gimbal angle
```

**Phase 3 Descent Aggression**:
```python
# Increase safety factor to decelerate harder:
# In phase3_controller():
a_required = (vz**2 / (2*z)) * 1.5   # 1.5 = current (50% safety)
                                      # 1.7 = more aggressive
                                      # 1.2 = more gentle
```

**LQR Attitude Gains**:
```python
LQR_Q = diag([0.1, 0.1, 500, 500, 100, 100])
#                  ↑    ↑    ↑    ↑    ↑    ↑
#                  vx   vy   θ_e  ψ_e  q    r
#               Higher values = tighter control
# Current: θ/ψ errors weighted 500 → very aggressive attitude
#          q/r rates weighted 100 → strong damping
```

---

## Debugging Checklist

### Phase 1c Not Executing?
1. Check apogee detection: `if vz <= 0.0:` in phase1b check
2. Verify frame counter: `if self.frame_count % 50 == 0:` debug prints
3. Check console for "ENTERING PHASE 1c NOW!" message
4. If missing, trajectory might not reach apogee (z too low)

### Gimbal Angles Too Small?
1. Verify throttle: should be 50% (PHASE1C_THROTTLE = 0.50)
2. Check KD value: should be 50,000 (PHASE1C_FLIP_KD)
3. Compute expected: at 50% = 150kN, L = 2.8m
   - Max torque = 50,000 N⋅m
   - Max gimbal = 15° = 0.26 rad
   - Energy from KD: F·L = 420,000 N⋅m, so τ/(F·L) = 0.12 rad at max
4. If gimbal still <0.1°, increase KD to 100,000

### Landing Velocity Too High?
1. Check ignition altitude: print Z and d_stop during phase 2
2. Verify safety factor: phase3_controller() line with `* 1.5`
3. Increase factor to 2.0 for more aggressive burn
4. Check LQR gains: if attitude not tight, velocity won't dampen

### Visualization Gimbal Wrong Direction?
1. Check exhaust direction in plotter.py line 166:
   ```python
   exhaust_dir_body = np.array([delta_y, delta_z, -1.0])  # ✓ Correct
   ```
   Should NOT be: `[-delta_y, -delta_z, -1.0]`
2. Verify both local view (line 166) and global view (line 119)

### Animation Jerky or Slow?
1. Reduce SIM_DT in config.py (smaller = more precision but slower)
2. Reduce point density on trajectory
3. Disable telemetry text box (heavy rendering)
4. Use lower speed factor (0.5x) in playback

---

## Common Control Law Implementations

### How to Implement a New Phase Controller

**Template**:
```python
def new_phase_controller(state, rocket, time_in_phase=0.0):
    """
    New phase controller.
    
    Args:
        state: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]
        rocket: Rocket object
        time_in_phase: Time since entering phase (seconds)
    
    Returns:
        {
            "throttle": float [0, 1],
            "gimbal_y": float radians,
            "gimbal_z": float radians,
            "aero_torque": [3] body-frame torque (N⋅m),
            "grid_fins": float [0, 1] (deployment),
            "leg_deploy": float [0, 1] (for landing legs)
        }
    """
    pos = state[0:3]
    vel = state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]
    
    # Your control logic here
    throttle = 0.5
    delta_y = 0.0
    delta_z = 0.0
    aero_torque = np.zeros(3)
    
    return {
        "throttle": throttle,
        "gimbal_y": delta_y,
        "gimbal_z": delta_z,
        "aero_torque": aero_torque,
        "grid_fins": 0.0,
        "leg_deploy": 0.0,
    }
```

**Then add to simulator.compute_control()**:
```python
elif self.phase == "new_phase":
    control = new_phase_controller(self.state, self.rocket, ...)
```

---

## State Vector Breakdown

```
state = [x,   y,   z,   vx,  vy,  vz,  phi, theta, psi, p,   q,   r  ]
         ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑      ↑    ↑    ↑    ↑
         0    1    2    3    4    5    6    7      8    9    10   11
         
    Position (world frame)
    ├─ x: Downrange distance (m)
    ├─ y: Lateral distance (m)
    └─ z: Altitude above ground (m)
    
    Velocity (world frame)
    ├─ vx: Downrange velocity (m/s)
    ├─ vy: Lateral velocity (m/s)
    └─ vz: Vertical velocity (m/s, negative = falling)
    
    Euler Angles (ZYX convention)
    ├─ phi (φ): Roll angle, 0 = level
    ├─ theta (θ): Pitch angle, 0 = upright, π = inverted
    └─ psi (ψ): Yaw angle, 0 = downrange aligned
    
    Angular Rates (body frame)
    ├─ p: Roll rate (rad/s)
    ├─ q: Pitch rate (rad/s)
    └─ r: Yaw rate (rad/s)
```

---

## Physics Quick Reference

### Gimbal Torque Calculation

```
Gimbal deflection angles: δy, δz (small, radians)
Thrust: F = throttle × max_thrust (Newtons)
Lever arm: L = CG_FROM_GIMBAL = 2.8 m

Thrust vector (body frame):
  F_body = [F·δy, F·δz, F]  (last component is vertical thrust)

Gimbal location relative to CG:
  r_gimbal = [0, 0, -L]  (engine is L below CG along -Z)

Torque due to gimbal thrust:
  τ = r_gimbal × F_body
    = [0, 0, -L] × [F·δy, F·δz, F]
    = [-L·F·δz, +L·F·δy, 0]  (coupling: gimbal deflection causes torque)

Angular acceleration:
  ω̇ = I⁻¹ @ τ  (smaller I = faster rotation)
  
Example: At 300kN, L=2.8m, δy=1° (0.0175 rad):
  τ_y = 2.8 × 300000 × 0.0175 = 1,470,000 N⋅m
  ω̇_q = τ_y / I_yy = 1,470,000 / 68,400 ≈ 21.5 rad/s²
```

### Coordinate Transformations

```python
# Body → World
v_world = DCM @ v_body  # DCM = euler_to_dcm(phi, theta, psi)

# World → Body
v_body = DCM.T @ v_world  # Transpose = inverse for orthogonal matrices

# Rocket nose direction in world
u_nose = body_z_axis_in_world(phi, theta, psi)  # = DCM[:, 2]

# Euler rate kinematics (body rates → Euler angle derivatives)
euler_dot = omega_to_euler_rates(phi, theta, omega)
# [φ̇, θ̇, ψ̇] = f(φ, θ, [p, q, r])
```

---

## Testing & Validation

### Unit Test Example: Gimbal Direction

```python
# Verify gimbal deflection produces correct thrust vector
delta_y = 0.1  # 5.7° right deflection
delta_z = 0.0
F = 150000     # 50% throttle

F_body = np.array([F*delta_y, F*delta_z, F])
print(f"Thrust body: {F_body}")
# Expected: [15000, 0, 150000]  (thrust tilts right)

# Gimbal torque
L = 2.8
tau = np.cross([0, 0, -L], F_body)
print(f"Torque: {tau}")
# Expected: [0, 420000, 0] (positive pitch torque - nose up)
```

### Integration Test: Phase 1c Convergence

```python
# Verify vector-based flip reaches retrograde
u_rocket = np.array([0, 0, 1])   # Nose up initially
vel = np.array([100, 0, -1000])  # Mostly downward
u_desired = -vel / np.linalg.norm(vel)

print(f"u_desired: {u_desired}")  # Should be ~[0, 0, 1] (retrograde)
print(f"Angle error: {np.arccos(np.dot(u_rocket, u_desired)):.2f} rad")  # ~π (180°)

rotation_axis = np.cross(u_rocket, u_desired)
print(f"Rotation axis: {rotation_axis}")  # Axis perpendicular to both
```

---

## Performance Tuning

### Phase 1c Flip Time
```
Typical: 2-5 seconds

If too long (>8s):
  ↳ Increase PHASE1C_FLIP_KD (gimbal response too slow)
  ↳ Increase PHASE1C_THROTTLE (less control authority)
  ↳ Reduce PHASE1C_FLIP_SUCCESS_RATE (relax stabilization threshold)

If too fast (<1s):
  ↳ Decrease PHASE1C_FLIP_KD (gimbal too aggressive)
  ↳ You want smooth convergence, not bang-bang
```

### Phase 3 Descent Time
```
Typical: 3-8 seconds to land from 100m apogee

If too fast (<2s):
  ↳ Decrease safety factor (1.5 → 1.2)
  ↳ Reduce LQR_Q attitude weights (smoother descent)
  
If too slow (>10s):
  ↳ Increase safety factor (1.5 → 1.8)
  ↳ Increase LQR_Q (tighter attitude control = more aggressive)
```

---

## Known Limitations & Caveats

1. **No propellant mass loss**: Mass constant at 10,000 kg
   - Fix: Add mass_dot parameter, update config.MASS dynamically

2. **Artificial roll damping**: Phase 3 uses 5000 N⋅m⋅s damping instead of RCS
   - Fix: Implement actual reaction control thrusters

3. **Grid fin modeling**: Grid fins deployed but not fully modeled
   - Current: Aerodynamic forces only
   - Missing: Fin deployment dynamics, panel breakaway

4. **Sensor noise**: No IMU noise, GPS noise, or filter lag
   - Fix: Add measurement noise and Kalman filter

5. **Structural loads**: No stress analysis or max load constraints
   - Fix: Track gimbal torques, compute stress factor

6. **Gust/wind**: Only gravity + thrust + aerodynamic damping
   - Fix: Add wind disturbance model

---

## Key References in Code

### Where to Find...

| What | File | Function | Line |
|------|------|----------|------|
| Phase transitions | simulator.py | `_update_phase()` | ~75-150 |
| Phase 1a/1b control | cascaded.py | `pure_pursuit()`, `outer_loop()`, `middle_loop()`, `inner_loop()` | ~30-100 |
| Phase 1c vector control | cascaded.py | `phase1c_controller()` | ~175-280 |
| Phase 2 flip control | cascaded.py | `phase2_flip_controller()` | ~150-175 |
| Phase 3 LQR control | cascaded.py | `phase3_controller()`, `lqr_gain()` | ~280-410 |
| Physics integration | dynamics.py | `state_derivative()` | ~1-70 |
| Gimbal-to-torque | dynamics.py | (cross product computation) | ~45-50 |
| Visualization update | plotter.py | `update()` | ~65-250 |
| Landing leg animation | plotter.py | (leg deployment loop) | ~180-210 |
| Landing detection | simulator.py | (Z < 0.5m check) | ~190-210 |

---

**Last Updated**: Session 6 (Final Documentation Phase)
**System Status**: ✅ Complete and operational

