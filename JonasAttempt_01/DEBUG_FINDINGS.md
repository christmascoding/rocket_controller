# Comprehensive Debug Analysis - Control Architecture Findings

## The Critical Bug Discovered (Iteration 12)

### Symptom
Simulation was crashing with gimbal saturated at ±5°, pitch diverging to 100°+

### Root Cause Analysis (Using Comprehensive Debug Output)

When inspecting the complete control signals, we discovered the **fundamental control architecture flaw**:

```
DES_ATT:[P 0.0°,Y 0.0°]   <- Body should be level
REF_ACC:[0.50, -0.62]m/s² <- But trajectory expects ascent!
GIMBAL cmd: [3.11°, 15.01°] <- Gimbal trying to pitch up
att: [3.1°, ...] <- Attitude already pitched...
```

**The controller was commanding:**
- Desired pitch = 0° (stay level)
- Gimbal pitch = 3.11° (maneuver)
- **Result: Attitude control vs. Gimbal control FIGHTING each other!**

### Why This Happened

1. **MPC disabled** → desired_acc = [0, 0, 0]
2. **Gravity compensation:** desired_thrust_vec = mass × (0 - gravity) = vertical thrust
3. **Desired attitude:** pitch_yaw_from([0,0,1]) = 0° pitch, undefined yaw
4. **But gimbal** was still computing: "point thrust in body frame" → non-zero gimbal commands
5. **Attitude control** tried to level the rocket while **gimbal control** tried to maneuver it
6. **Result:** Control loop oscillation with gimbal saturation → instability → crash

## The Fix: Architectural Consistency

### Change Made
When MPC is disabled (gravity compensation mode):
```python
if not self.config.mpc.enable_mpc:
    gimbal_pitch = 0.0  # Zero gimbal - no maneuvering
    gimbal_yaw = 0.0    # Pure gravity compensation only
```

### Why This Works
- **Desired attitude:** 0° pitch (point straight up)
- **Gimbal commands:** 0° (neutral)
- **Attitude control:** Maintains body level
- **Gimbal control:** Maintains neutral deflection
- **No fighting!** ✅ Single unified control objective

## Design Lesson

### The Core Insight
Control architectures must have **consistent objectives across all layers**:

1. **Upper layer** defines desired acceleration
2. **Middle layer** converts to desired attitude
3. **Lower layer** (gimbal) implements that attitude
4. **All layers must agree on the same goal**

### What NOT To Do
❌ Command level attitude (0° pitch) while gimbal tries to maneuver
❌ Use gimbal for trajectory tracking when attitude control is trying to stabilize
❌ Mix implicit objectives (gimbal maneuver) with explicit ones (zero pitch)

### What TO Do  
✅ When MPC disabled: Pure gravity compensation (zero gimbal)
✅ When MPC enabled: Let MPC control both desired_acc AND gimbal through unified optimizer
✅ Attitude control: Always a **supporting layer** that stabilizes the body

## Debug Output Lessons

### What The Comprehensive Debug Revealed
The new debug monitor shows **ALL control signals in real-time**:

```
State:           Position, velocity, attitude, rates
Commands:        Desired acceleration, desired pitch/yaw, gimbal command
Actuators:       Thrust command vs. actual, gimbal command vs. actual  
Errors:          Position error, attitude error, tracking error
Energy:          Kinetic + potential for energy analysis
Reference:       Trajectory reference position/velocity/acceleration
```

### How To Use This For Future Debugging
1. **Check attitude errors** first - if non-zero, attitude control isn't tracking
2. **Check gimbal saturation** - if saturated, control authority insufficient
3. **Check control consistency** - desired attitude should match desired acceleration direction
4. **Check thrust** - if less than command, actuator rate-limited
5. **Check energy** - should conserve or increase based on thrust

## Test Results

### Iteration 12 (Before Fix)
- Crashed at t ≈ 14.8s with gimbal saturated
- Pitch diverged 0° → 109.2°
- Position error grew unbounded

### Iteration 12 (After Fix)  
- ✅ Completed full 150 seconds
- ✅ Gimbal maintained at 0° (neutral)
- ✅ Attitude stable at 0° pitch (level cruise)
- ✅ Speed maintained constant (24.8 m/s)
- ✅ Zero angular rates - perfectly stable

## Recommendations

### For MPC Implementation
1. **Always make gimbal/actuators part of MPC** - don't add manual gimbal commands
2. **Let optimizer decide** attitude and gimbal together
3. **Use proportional attitude control only** - avoid derivatives that cause instability

### For Testing New Configurations
1. **Enable comprehensive debug** FIRST
2. **Watch the three key consistency checks:**
   - Desired attitude matches thrust direction? 
   - Gimbal commands consistent with MPC objectives?
   - Attitude control errors trending to zero?
3. **If any answer is "No"**, there's a control architecture bug

### For Iteration 13+
Consider: **Is the problem in the control law or in the trajectory?**
- Rocket can maintain 25 m/s level flight indefinitely (proven)
- Question: What trajectory is physically achievable with this control authority?
