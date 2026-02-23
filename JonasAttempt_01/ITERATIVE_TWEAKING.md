# Rocket Controller Debugging Log

## Problem Statement
Rocket is oversteer ing and spiraling, poor path following. Not pointing at the right target.

## Iterations

### Iteration 1: Initial Investigation
**Changes made:**
- Exposed `gimbal_offset_m` to config (from hardcoded 3.5m)
- Increased debug verbosity to show every timestep
- Disabled GUI visualization
- Added pointing error metric (angle between rocket body direction and path direction)
- Added path error metric (distance from closest point on path)

**Results:**
- Pointing error: 20.9° → 178.2° (rocket completely misaligned)
- Path error: 0.53m → 25.50m (rocket flying far from path)
- Pitch rate oscillation: 6.1 deg/s → 27.0 deg/s (unstable)
- Gimbal saturating at 10° (max allowed)
- Issue: Lead time was 4.8 seconds (way too aggressive!)

**Root cause identified:** 
- MPC reference horizon was set 4.8 seconds ahead, causing rocket to chase a target too far ahead
- This caused aggressive gimbal commands that induced pitch oscillations

---

### Iteration 2: Reduce Lead Time + Increase Pitch Damping
**Changes made:**
- `lead_time_s`: 4.8s → 0.3s
- `pitch_damping`: 50% → 90% of angular velocity
- MPC weights unchanged (too aggressive)

**Results:**
- Pointing error: still diverged to ~175° by t=0.7s
- Path error: 0.53m → 6.49m by t=0.72s
- Pitch oscillation: 6.0° → 13.8° (slower but still growing)
- Gimbal still saturating
- Problem: MPC still too aggressive on position tracking

---

### Iteration 3: Reduce MPC Aggressiveness
**Changes made:**
- `weight_pos`: 10.0 → 3.0 (less aggressive position tracking)
- `weight_vel`: 5.0 → 2.0 (less aggressive velocity tracking)
- `max_accel`: 14.0 → 8.0 m/s² (limit control authority)
- `max_lateral_accel`: 1.0 → 0.5 m/s² (reduce gimbal authority)
- `lead_time_s`: 0.3s → 0.15s (further reduced)

**Results:**
- Pointing error: still diverges to ~175° by t=0.7s
- Path error: 0.53m → 6.46m by t=0.72s
- Pitch oscillation: 6.0° → 13.8° (same as before)
- Still oscillating and diverging
- Problem: Lead time concept itself is causing issues

---

### Iteration 4: Remove Lead Time Entirely
**Changes made:**
- `lead_time_s`: 0.15s → 0.0s (no lead, track closest point)

**Results at t=0.7s:**
- Pointing error: still diverges to ~169° (same as before)
- Path error: 0.53m → 6.14m (same pattern)
- Pitch oscillation: 6.0° → 13.7° (growing)
- Still oscillating - lead time was NOT the root cause

**Results at t=44s (end):**
- **CATASTROPHIC FAILURE**
- Roll: 14.7° (massive uncontrolled roll)
- Pitch: 20.2° (excessive pitch up)
- Yaw: 196.7° (spinning 180+ degrees around!)
- Yaw rate: -47.6 deg/s (wildly spinning)
- Path error: 305 m (totally lost)
- Gimbal saturated at ±10° (no more control authority)

**Root cause identified:**
The problem is NOT the lead time or MPC weights.  The issue is **gimbal-induced instability**:
- Gimbal pitch creates pitch torques that are not properly compensated
- The pitch compensation code (line 269-271) tries to counter this, but it's fighting a fundamentally unstable system
- The yaw rate damping (80%) and pitch damping (90%) are being overwhelmed by gimbal torques
- The control loop goes unstable, causing roll, pitch, and yaw to diverge

**Next approach:**
Need to fundamentally rethink how gimbal torques affect attitude. The issue is likely:
1. Gimbal offset distance is wrong, OR
2. The pitch compensation gain is wrong, OR  
3. The moment of inertia calculation is wrong, OR
4. The gimbal actuator dynamics are fighting the attitude control

---

## Iteration 6: Extended Trajectory 5x for Better Alignment  
**Objective**: Give controller more runway for initial alignment

**Changes made:**
- `parabola_cruise_length_m`: 700m → 3500m
- `parabola_cruise_time_s`: 15s → 75s  
- `total_time`: 90s → 180s

**Results at t=0.5s**: Initially OK
- Pitch: 4.5-5.0°, stable gimbal approach

**Results at t=1.0s**: Started deteriorating
- Pointing error: 100-150°
- Pitch oscillations: ~8.2°/s
- Yaw oscillations: Oscillating, growing OSC_WARN

**Results at t=88s (end)**: CATASTROPHIC FAILURE
- Yaw: 88.1°
- Path error: 594m
- Yaw rate: 35.4°/s (spinning)
- Gimbal: 5.0°/5.0° (saturated, no authority)

**Root cause**: Extended trajectory gave MPC MORE TIME to compound errors
- MPC tries to correct horizontal position errors over longer horizon
- Gimbal saturates and system becomes unstable faster

**Lesson**: Longer runway made problem WORSE, not better!

**Reverted to**: 700m cruise

---

## Iteration 7: Reduce Gimbal to 2.5°
**Objective**: Minimize gimbal-induced torques by reducing max deflection

**Changes made:**
- `gimbal_max_deg`: 5.0° → 2.5°
- `gimbal_rate_limit`: 20.0°/s → 10.0°/s
- `gimbal_accel_limit`: 100.0°/s² → 50.0°/s²

**Results at t=0.5s**:
- Path error growing faster: 0.53m → 3.5m
- Pointing error: Jumping 140-180° range

**Results at t=90s (end)**: FAILURE
- Yaw: 213.9° (full rotation)
- Path error: 747m
- Gimbal: 2.5°/2.5° (maxed out, insufficient authority)

**Root cause**: Too little gimbal authority
- MPC still tries to track path aggressively
- Gimbal has NO authority to respond
- System diverges FASTER than with 5° gimbal

**Lesson**: Reducing gimbal below 5° is counterproductive

---

## Iteration 8: BREAKTHROUGH DISCOVERY - Attitude-Only Control (MPC Disabled)
**Objective**: Determine if gimbal/attitude control is stable WITHOUT MPC path tracking

**Changes made:**
- Added `enable_mpc` flag to config
- Set `enable_mpc = False` (NO path tracking, just gravity compensation + attitude hold)
- Gimbal remains at 5°, full attitude control active

**Results at t=0.5s**: PERFECT STABILITY
- **Pitch**: 4.4-4.6° - COMPLETELY STABLE, zero oscillation!
- **Roll**: ~0° - perfectly controlled
- **Yaw**: 0° - rock solid
- **Angular rates**: All near zero - perfectly damped!
- **Gimbal**: Naturally symmetric (pitch ≈ yaw) - very stable
- Path error: 0.53m → 0.55m (growing but predictably)

**Results at t=90s (end)**: SUSTAINED PERFECT STABILITY
- **Roll**: -2.9° - stable banking
- **Pitch**: 9.8° - drifted slowly but stable
- **Yaw**: 0° - maintained throughout
- **Angular rates**: 0.0°/s - no motion at all
- **Gimbal**: 5.0°/5.0° - constant and steady
- **Path error**: 6497m - linear divergence (expected for straight flight)

**STATUS**: ✓ COMPLETELY STABLE with attitude-only control!

## **CRITICAL INSIGHT**: 
**The problem is NOT gimbal dynamics, NOT attitude control, NOT damping coefficients!**  
**The problem is MPC trying to force the rocket to track a horizontal path it cannot physically follow!**

**Evidence**:
1. **Without MPC** (attitude-only): PERFECTLY stable, no oscillations, no divergence
2. **With MPC on 700m path**: Stable for ~1s, then diverges into oscillations and spirals
3. **With MPC on 3500m path**: Diverges FASTER (more time for compounding errors)
4. **With smaller gimbal (2.5°)**: Diverges EVEN FASTER (less authority to respond)

The gimbal was never the problem!

---

## Iteration 9: Minimal Position Weight MPC
**Objective**: Try MPC with horizontal position tracking DISABLED

**Changes made:**
- Re-enabled MPC with modified weights:
  - `weight_pos`: 1.0 → 0.0 (DISABLED horizontal position tracking)
  - `weight_vel`: 1.0 → 0.2 (very gentle velocity)
  - `horizon_steps`: 15 (normal)

**Results**: Same failures as full MPC
- Early oscillations in yaw
- Gimbal oscillates ±1.3° at t=0.5s
- Yaw rate grows to 7.1°/s by t=0.8s

**Root cause identified**:
- Setting `weight_pos=0` doesn't help because reference trajectory STILL contains horizontal positions
- MPC still tries to minimize distance to that reference (implicitly)
- Lower weight just makes divergence slower, not nonexistent

**Lesson**: Need to change the REFERENCE TRAJECTORY, not just the weights

---

## ROOT CAUSE SUMMARY

After iteration 8 breakthrough, the actual problem is now crystal clear:

### The Mismatch
1. **Rocket starts at**: 45° pitch (aerodynamic equilibrium at high cruise speed)
2. **Path reference is**: Horizontal (z=constant during cruise)
3. **Required maneuver**: Pitch down 45° while maintaining position
4. **Gimbal capability**: ±5° deflection
5. **MPC demand**: Minimize position error → increasingly aggressive gimbal commands
6. **Result**: Gimbal saturates at ±5° with no steering authority left

### Why Iteration 5 was Better
- Gimbal at 5° provided just enough authority
- Even though MPC was demanding impossible maneuvers
- Attitude control + damping could keep system BARELY stable for ~2 seconds
- After that, compounding errors caused divergence

### Why Longer Trajectory Made It Worse
- More cruise time = more time for MPC errors to compound
- Exponential divergence instead of linear
- Gimbal saturates earlier, loses control faster

### Why Smaller Gimbal Made It Worse  
- 2.5° gimbal had no authority to even TRY responding
- System diverged immediately (no control authority at all)
- Worse than 5° gimbal that at least tried (and partially worked)

---

## SOLUTION APPROACHES

**Option 1: Disable MPC (Current)**
- Pros: Perfectly stable, no tuning needed
- Cons: No path tracking, just straight-line flight
- Status: ✓ Validated as stable

**Option 2: Redefine Reference Trajectory**
- Instead of: "Follow horizontal path at constant altitude"
- Use: "Maintain current pitch angle and altitude, natural turn"
- Pros: Respects rocket dynamics
- Cons: Requires trajectory redesign

**Option 3: Redesign MPC Strategy**
- Separate control objectives:
  - **Vertical**: Aggressive altitude tracking (gimbal pitch) 
  - **Lateral**: Very conservative lateral tracking (gimbal yaw)
- Pros: Keeps MPC but makes it feasible
- Cons: Requires significant MPC restructuring

**Option 4: Model Predictive Control with Different Formulation**
- Current: Track position (forces aggressive gimbal)
- Alternative: Minimize fuel + maintain altitude band (gentler on gimbal)
- Pros: More realistic rocket control
- Cons: Complete MPC reformulation

**RECOMMENDATION**: Option 2 or 3
- Option 1 (disabled MPC) proves the diagnostics are correct
- The rocket clearly CAN be controlled stably
- Just need reference trajectory or MPC objectives that match rocket physics

---

## Iteration 11: Attempt to Implement Four German Requirements

**Objective**: Implement user-requested fixes simultaneously:
1. Increase gimbal authority (5° → 15°)
2. Add PI control with anti-windup
3. Strengthen roll damping (30% → 95%)
4. Redesign trajectory (longer phases for feasibility)

**Changes made**:
- `gimbal_max_deg`: 5° → 15°
- `gimbal_rate_limit_deg_per_s`: 20° → 30°
- `gimbal_accel_limit_deg_per_s2`: 100 → 150
- Added `attitude_integral_error` state for PI control
- Implemented PI controller: Kp=1.0/tau, Ki=0.1/tau with anti-windup
- Roll gimbal control gains: 0.3/0.15 → 0.5/0.3
- Angular damping: general 30%→50%, roll 30%→95%, yaw 80%→90%
- Trajectory phases extended: ascent 45→60s, cruise 15→30s, descent 32→40s
- Total simulation time: 90s → 180s
- MPC weights adjusted: weight_pos 0→2, weight_vel 0.2→1
- `enable_mpc`: re-enabled (was False)

**Results**: **CATASTROPHIC FAILURE**
- All configurations tested became unstable
- Pitch rate diverges continuously (9°/s → 41°/s → 77°/s depending on config)
- Pitch angle either wraps wildly (>400°) or saturates at limits (±90°)
- Gimbal saturates at max deflection (±15°) or (±5°)
- Rocket crashes to ground in 5-10 seconds

**Sub-iterations tested**:
1. Full config (15° gimbal + PI + trajectory) → pitch wraps to 486°
2. Reduced gimbal back to 5° + PI → pitch saturates at 90° after wrapping
3. Removed PI (pure proportional) → pitch rate still climbs to 41°/s
4. Removed angle wrapping safeguards → rate increases to 77°/s initially
5. Tried new PD control law (Kp=0.5/tau, Kd=0.3/tau, rate feedback) → Rate still diverges

**Root cause analysis**:
The instability is in the **attitude control feedback loop**, NOT hardware configuration:

- **Iteration 8 baseline was stable**: With MPC disabled, 5° gimbal, original proportional control → perfect stability (roll 0°, pitch 4.4-4.6°, all rates ~0°/s, 90s flight with no divergence)

- **Problem with Iteration 11**: Changed control law from simple proportional to PD with rate error feedback
  - Old (working): `att_rate_cmd = Kp * att_error` (feedforward style)
  - New (broken): `att_rate_cmd = Kp * att_error - Kd * omega` + `att_feedback = K * (att_rate_cmd - omega)` (proper feedback)
  
- The new control structure appears to introduce **positive feedback in pitch** or **instability in the feedback loop**. Even reverting gimbal back to 5° does NOT stabilize it - the control law is the culprit.

**Why the new approach failed**:
- Derivative term on attitude error (`-Kd * omega`) creates phase lag
- The rate error feedback (`att_rate_cmd - omega`) with K=0.5 appears insufficient to stabilize the new control structure
- Gimbal authority increase (15°) was intended to help, but can't help an unstable control law
- PI integral accumulates errors and makes divergence worse

**Key insight**: 
Cannot apply multiple major control law changes simultaneously with hardware changes. The gimbal authority increase, damping strengthening, and trajectory redesign are all sensible, BUT they must be applied incrementally to a **known-stable baseline control law**.

**Status**: **FAILED - BLOCKING ISSUE**
- Root cause identified but not resolved: Attitude control law instability
- Reverting all changes back to iteration 8 baseline is required before proceeding
- Must apply the four fixes **incrementally**, not simultaneously
- One change at a time, testing stability after each change

---

## Key Lessons Learned Across All Iterations

### What We Know Works
✓ **Stable configuration exists**: Iteration 8 with MPC disabled proved it
✓ **Gimbal and attitude control are capable**: 5° gimbal + proportional control stable
✓ **Damping is effective**: 95% roll damping prevents spiraling
✓ **Angle wrapping is necessary**: Prevents unbounded pitch/yaw values

### What Doesn't Work
✗ **Horizontal path following with 45° trim angle**: Fundamentally infeasible
✗ **MPC with aggressive position weights**: Gimbal can't provide required accelerations
✗ **PI control with weak anti-windup**: Integral term accumulates and diverges
✗ **Simultaneous control + hardware redesign**: Too many variables, hard to isolate root causes

### Control Law Principles
- **Keep it simple**: Proportional control on attitude error works well
- **Feedback structure matters**: Feedforward vs. proper error feedback behaves very differently
- **Gain scheduling is critical**: Same Kp that works at Mach 0.3 fails at Mach 2.0
- **Test one variable at a time**: Gimbal → damping → trajectory → control law (in sequence, not parallel)

