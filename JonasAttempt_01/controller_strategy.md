# Trajectory Controller Stabilization Strategy

## Goal
Make the rocket track the full trajectory (ascent → cruise → descent → landing) without pitch runaway or gimbal saturation instability.

## Diagnosis (Root Cause)
- The control law was using **wrong sign conventions** (state error = actual − reference), which flipped the feedback direction.
- Gimbal commands were issued as **position commands** without respecting the **20°/s rate limit**, causing phase lag and instability.
- The control architecture lacked **bandwidth separation**, so fast gimbal dynamics were fighting slow trajectory dynamics.

## Strategy
### 1) Fix Sign Conventions (Critical)
Use:
- $e_p = p_{ref} - p_{actual}$
- $e_v = v_{ref} - v_{actual}$

All feedback should be based on $e_p, e_v$ (not the raw state_error which is actual − reference).

### 2) Implement Cascade Control With Bandwidth Separation
**Outer loop (slow):** Position → velocity correction
- $v_{cmd} = K_p e_p$

**Middle loop (medium):** Velocity → attitude setpoint (pitch/yaw)
- $v_{err}^* = e_v + v_{cmd}$
- $\theta_{cmd} = K_v \cdot v_{err,x}^*$
- $\psi_{cmd} = K_v \cdot v_{err,y}^*$
- Clamp $\theta_{cmd}, \psi_{cmd}$ to safe limits

**Inner loop (fast):** Attitude → gimbal **rate** command
- $\dot{g}_{cmd} = K_{att}(\theta_{cmd}-\theta) - K_{rate}\cdot\dot{\theta}$
- Rate-limit $\dot{g}_{cmd}$ to 20°/s, then integrate to gimbal angle

### 3) Vertical Thrust Controller
- $T = T_{hover} + K_T (e_{v,z} + K_{p,z} e_{p,z})$
- Clamp $T$ to actuator limits

### 4) Keep Actuator Limits Consistent
- Gimbal angle limit = ±15°
- Gimbal rate limit = 20°/s (0.2° per 0.01s)

### 5) Validation
- Confirm pitch stays bounded (no multi-rotation spiral)
- Confirm landing near ref position and velocity at end of trajectory
- Confirm gimbal not saturating from start to end

## Implementation Steps
1. Update `src/controllers/lqr.py` to:
   - Correct sign conventions
   - Add cascade loops
   - Apply gimbal rate limiting and integration
2. Update `src/config.py` LQR tuning values to match the new cascade controller
3. Run `run_demo.py` and confirm stability and tracking

## Acceptance Criteria
- Pitch remains within ±45° during the entire flight
- Final position error < 20 m
- Final vertical velocity magnitude < 5 m/s
- No constant gimbal saturation from t=0 onward
