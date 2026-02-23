# Comprehensive Debug Monitor - Output Guide

## Overview
The new `DebugMonitor` class provides **COMPLETE VISIBILITY** into every control signal and state variable in the rocket controller.

## Output Format

Each debug line shows 4 sections with different groups of signals:

### Section 1: Time and Position/Velocity
```
t=  0.04s | POS:[    0.0,    1.0]m | VEL:[   0.0,  25.0]m/s | 
```
- `t`: Simulation time (seconds)
- `POS:[x, z]`: Horizontal and vertical position (meters)
- `VEL:[vx, vz]`: Horizontal and vertical velocity (m/s)

### Section 2: Attitude and Angular Rates
```
ATT:[R  -0.0°,P   3.1°,Y    0.0°] |
REF_POS:[    0.0,    0.5]m | ERR_POS:    0.5m | ERR_VEL:  0.01m/s |
```
- `ATT:[R, P, Y]`: Roll, pitch, yaw (degrees)
- `REF_POS`: Reference trajectory position  
- `ERR_POS`: Distance from reference position (meters)
- `ERR_VEL`: Velocity error magnitude (m/s)

### Section 3: Thrust and Gimbal Control
```
THRUST: cmd=   490N, actual=   463N | 
GIMBAL: cmd=[  3.11°, 15.01°] | actual=[  0.12°,  0.12°]
```
- `THRUST cmd`: Commanded thrust (Newtons)
- `THRUST actual`: Actual thrust after actuator dynamics
- `GIMBAL cmd`: Commanded gimbal angles [pitch, yaw] (degrees)
- `GIMBAL actual`: Actual gimbal deflection after rate limiting

### Section 4: Attitude Control and Acceleration
```
DES_ATT:[P   0.0°,Y    0.0°] | ERR_ATT:[P  -3.1°,Y    0.0°] | 
ANGVEL:[R  -0.0°/s,P  -0.1°/s,Y   0.0°/s]
```
- `DES_ATT`: Desired pitch and yaw from control law
- `ERR_ATT`: Attitude error (desired - actual)
- `ANGVEL`: Body angular velocity rates (deg/s)

### Section 5: Acceleration and Energy
```
DESIRED_ACC:[  0.00,  0.00]m/s² | REF_ACC:[  0.50, -0.62]m/s² | 
ENERGY: KE=   15596J, PE=     490J
```
- `DESIRED_ACC`: MPC/controller command [ax, az]
- `REF_ACC`: Reference trajectory acceleration
- `ENERGY`: Kinetic and potential energy (Joules)

## Key Signals to Watch

### Control Consistency Checks

**1. Attitude Error Should Go To Zero**
```
ERR_ATT:[P  -3.1°,Y    0.0°]  <- At t=0
ERR_ATT:[P   0.0°,Y    0.0°]  <- Should converge to zero
```
If this grows or oscillates wildly → attitude control problem

**2. Gimbal Should Match Control Objective**
```
DES_ATT:[P   0.0°,Y    0.0°]  <- Want level
GIMBAL cmd=[  0.00°,  0.00°]  <- Gimbal should be neutral
```
If gimbal commands don't match desired attitude → control law bug

**3. Actual Should Track Command**
```
THRUST: cmd=   490N, actual=   463N  <- Gap due to actuator lag (normal)
GIMBAL: cmd=[  3.11°, 15.01°] | actual=[  0.12°,  0.12°]  <- Tracking delay
```
If actual lags far behind or saturates → actuator limit hit

**4. Desired Acceleration Should Match Reference**
```
DESIRED_ACC:[  0.00,  0.00]m/s²  <- Zero (gravity comp)
REF_ACC:[  0.50, -0.62]m/s²      <- Non-zero (trajectory wants accel)
```
Large mismatch → either trajectory unrealistic or MPC not working

### Gimbal Saturation Warning
```
*** GIMBAL SATURATED ***  <- Appears if gimbal angles > ~5°
```
Indicates control authority insufficient for desired maneuver

## Real Example: Control Architecture Bug

**BEFORE FIX (Iteration 12 - Crashing):**
```
DES_ATT:[P   0.0°,Y    0.0°]         <- Want level
GIMBAL cmd=[  3.11°, 15.01°]         <- But gimbal trying to pitch!
att=[3.1°]                           <- Body fighting between controls
```
**Problem:** Attitude control trying to level, gimbal trying to maneuver = instability

**AFTER FIX (Stable):**
```
DES_ATT:[P   0.0°,Y    0.0°]         <- Want level
GIMBAL cmd=[  0.00°,  0.00°]         <- Gimbal neutral ✓
att=[0.0°]                           <- Body stays level ✓
```
**Solution:** Make gimbal consistent with attitude control objective

## Common Issues and Their Symptoms

| Issue | Symptoms in Debug Output |
|-------|-------------------------|
| **Gimbal fighting attitude** | DES_ATT ≠ GIMBAL cmd direction |
| **Actuator rate-limited** | `cmd` >> `actual` for long duration |
| **Control loop diverging** | ERR_ATT growing over time |
| **Trajectory unrealistic** | DESIRED_ACC ≈ 0 but REF_ACC large |
| **Gimbal saturated** | GIMBAL actual = ±5° constantly |
| **Energy going negative** | PE drops but thrust not matching |

## How To Enable Debug Output

Edit `src/config.py`:
```python
@dataclass
class DebugConfig:
    show_progress: bool = True
    progress_every_steps: int = 1      # Every step
    show_mpc_debug: bool = True
    mpc_debug_every_steps: int = 1     # Every step
```

Then run simulation - debug output appears every 2 steps (configurable in `DebugMonitor`).

## Using Debug Output For Iteration 13+

When implementing new features:

1. **Enable debug** first thing
2. **Run short test** (10-20 seconds)
3. **Check three things:**
   - Do attitude errors go to zero?
   - Do gimbal/thrust match control objectives?
   - Does energy make sense (increase with positive thrust)?
4. **If any fail:** Screenshot the debug output and analyze
5. **Common fix:** Check control law consistency

This debug visibility has saved hours of debugging by showing exactly where control loops are fighting or misaligned!
