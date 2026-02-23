# Iteration 11: Gimbal Authority Increase Attempt - Status Report

## Objective
Implement four fixes requested by user:
1. Increase gimbal authority (5° → 15°)
2. Add PI control with integral state and anti-windup
3. Strengthen roll damping (30% → 95%)
4. Redesign trajectory to be feasible (longer phases)

## Status: FAILED - UNSTABLE SYSTEM

### Problem Encountered
After implementing the four fixes, the system became unstable:
- Pitch rate diverges (climbs continuously)
- Pitch angle saturates (either at limits or grows unbounded)
- Gimbal authority gets saturated at max deflection
- System crashes within seconds of simulation

### Root Cause Analysis

The instability appears to be in the **attitude control feedback loop**, NOT in the hardware or configuration. Evidence:

1. **Iteration 8 baseline**: With MPC disabled, 5° gimbal, and original proportional control, system was PERFECTLY STABLE
   - Roll 0°, Pitch 4.4-4.6°, Yaw 0°
   - All angular rates near zero
   - No divergence over 90 second flight

2. **Current iteration issues**:
   - Even with 5° gimbal (reverted from 15°), new PD control law causes divergence
   - Removing PI integral logic but keeping new control law → still unstable  
   - Angle wrapping fixes caused pitch to clip at 90° instead of diverging
   - Problem persists across all configuration variations tested

3. **Control law is the culprit**:
   - Changed from simple proportional to PD with rate feedback
   - Changed attitude rate command application from feedforward to proper error-based feedback
   - These changes somehow introduced positive feedback or oscillation

### Configurations Tested (This Session)

| Config | Gimbal | MPC | Trajectory | Control Law | Result |
|--------|--------|-----|------------|-------------|--------|
| v1 | 15° | OFF | Extended | PI +  90% roll damp | **UNSTABLE**: Pitch >480°, Gimbal saturated |
| v2 | 15° | OFF | Original | Pure P (no integral) | **UNSTABLE**: Pitch rate → 41°/s |
| v3 | 15° | OFF | Original | PD + rate feedback | **UNSTABLE**: Pitch rate → 29°/s |
| v4 | 5° | OFF | Original | PD + rate feedback | **UNSTABLE**: Pitch →  90°, rate → 15°/s |
| v5 (current) | 5° | OFF | Original | PD + rate feedback (no wrap) | **UNSTABLE**: Pitch rate → 9°/s |

### Control Law Comparison

**Iteration 8 (STABLE)**:
```python
# Unknown exact code, but apparently:
# - Pure proportional on attitude error
# - Feedback applied as angular acceleration term
# - Conservative gains
# - Strong roll damping (95%)
# - Simple gimbal roll control
```

**Iteration 11 (UNSTABLE)**:
```python
# Current attempt:
Kp_att = 0.5 / tau       # Proportional on attitude error
Kd_att = 0.3 / tau       # Derivative on attitude error
att_rate_cmd = Kp_att * att_error - Kd_att * omega  # PD law

# Feedback application:
ang_vel_error = att_rate_cmd - omega
att_feedback = 0.5 * ang_vel_error   # Rate error feedback

# Damping:
angular_velocity_damping = -0.3 * omega  # 30% general
angular_velocity_damping[0] = -0.95 * omega[0]  # 95% roll
```

### What Changed
The biggest changes were:
1. **Added derivative term on attitude error** (`-Kd_att * omega`) - This could create phase lag issues
2. **Changed feedback from feedforward to error-based** - The `att_rate_cmd` is now compared against actual `omega` to create error
3. **Reduced general damping** from 50% back to 30% - Might be insufficient for new control structure

### Hypothesis
The PD control law with rate error feedback is creating **positive feedback loop** or **instability** in pitch control:
- When pitch rate is positive (pitch increasing), the `- Kd_att * omega[1]` term REDUCES the pitch command
- But this might not be the right negative feedback for the system dynamics
- The `att_feedback = 0.5 * ang_vel_error` then tries to correct, but with wrong sign or magnitude

### Next Steps to Resolve

1. **Option A: Revert to Iteration 8 control law**
   - Need to identify exact working code from iteration 8
   - Use git history or backup to restore proven stable configuration
   - Test with new gimbal/damping values only

2. **Option B: Simplify control law**
   - Remove derivative term on attitude error (just use P)
   - Use proper rate tracking feedback (omega_cmd - omega) with lower gain
   - Test incrementally with 5° gimbal first

3. **Option C: Stability analysis**
   - Apply linearized stability analysis to attitude dynamics
   - Verify feedback gains are in stable region
   - Check phase margins and damping ratios

4. **Option D: Add detailed logging**
   - Print attitude, angular_velocity, gimbal commands, forces every step
   - Trace the divergence path in detail
   - Identify exact moment instability begins

### Files Modified This Session
- `src/config.py`: Gimbal increased to 15°, trajectory extended, then reverted to 5° and original trajectory
- `src/simulation/simulator.py`: 
  - Added PI control (then removed as problematic)
  - Modified attitude error calculation
  - Added angle wrapping (then disabled)
  - Changed feedback application method
  - Increased damping values

###Code State
Current code is UNSTABLE and should not be used for actual control testing. System immediately diverges in pitch.

### Recommendation
**STOP and revert to iteration 8 configuration before making further changes.** The control law change is the root cause, and the four fixes (gimbal, trajectory, damping) should be applied incrementally to a known-stable baseline, not simultaneously with control law changes.

---
**Iteration**: 11  
**Status**: FAILED  
**Blocking Issue**: Attitude control instability  
**Recommendation**: Revert control law, apply fixes incrementally
