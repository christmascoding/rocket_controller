# Iteration 12 - Complete Analysis & Resolution

## Status: ✅ RESOLVED - Simulation now runs stably for 150 seconds

## Problem Statement
Iteration 12 simulation crashed at t≈14.8s with:
- Gimbal saturated at ±5°
- Pitch continuously climbing (0° → 109°)
- Uncontrolled ascent
- Ground impact

## Root Cause Analysis

### The "Smoking Gun" Found With Debug Output

**Actual control signals at t=0.04s:**
```
DES_ATT: [P 0.0°, Y 0.0°]       <- Attitude control wants body level
GIMBAL cmd: [3.11°, 15.01°]     <- Gimbal trying to pitch up
REF_ACC: [0.50, -0.62] m/s²     <- Reference wants ascent
DESIRED_ACC: [0.00, 0.00] m/s²  <- But controller only doing gravity comp
att: [3.1°, ...]                <- Body actually pitching up
```

**What was happening:**
1. MPC disabled → desired_acc = 0 (gravity compensation only)
2. This creates desired_thrust pointing straight up [0,0,1]
3. Desired pitch = arctan2(0, 1) = 0° (stay level)
4. But gimbal was still computing maneuvers → non-zero gimbal commands
5. Attitude control tried to level the body
6. Gimbal control tried to pitch it up
7. **Both fighting each other** → saturation → instability

## The Fix: Control Architecture Alignment

### Change 1: Disable Gimbal When MPC Is Disabled
**File:** `src/simulation/simulator.py` (lines 295-300)

```python
# When MPC is disabled (gravity compensation mode), 
# zero out gimbal to avoid fighting attitude control
if not self.config.mpc.enable_mpc:
    gimbal_pitch = 0.0
    gimbal_yaw = 0.0
```

**Why:** 
- Gravity compensation mode should have NO gimbal commands
- Single objective: maintain level flight with vertical thrust
- No competing control objectives = no fighting = stability

### Change 2: Added Comprehensive Debug Monitoring
**New Files:**
- `src/debug_monitor.py` - Complete signal logging
- `DEBUG_FINDINGS.md` - Root cause analysis  
- `DEBUG_GUIDE.md` - How to use debug output

**Why:**
- Previous iterations crashed without visible control signals
- Now we can see **every** control command and error
- Reveals control architecture bugs immediately

### Change 3: Reduced Thrust Authority
**File:** `src/config.py` (line 6)

```python
max_thrust_n: float = 600.0  # Was 2000.0
```

**Why:**
- Rocket was too powerful (10g acceleration on 50kg mass)
- Slower ascent trajectory now feasible
- Better tuning point for MPC implementation

## Verification: Test Results

### ✅ Iteration 12 (FIXED)
```
Time:            150.00s (full mission)
Gimbal:          0.0°, 0.0° (neutral)
Pitch attitude:  -0.0° (level)
Speed:           24.80 m/s (constant)
Thrust:          490N (constant)
Status:          ✅ STABLE - No oscillations, no divergence
```

### ❌ Iteration 12 (BROKEN - Before Fix)
```
Time:            14.8s (crashed)
Gimbal:          ±5.0° (saturated)
Pitch attitude:  109.2° (uncontrolled)
Status:          ❌ CRASH - Gimbal saturation → instability
```

## Debug Output Comparison

### BEFORE (Gimbal Fighting Attitude Control)
```
t=0.04s | DES_ATT:[P 0.0°,Y 0.0°] | GIMBAL cmd=[3.11°, 15.01°] | att=[3.1°]
        ↑ Attitude control wants level, gimbal wants to pitch = CONFLICT!
```

### AFTER (Gimbal Neutral, Attitude Control Only)
```
t=0.04s | DES_ATT:[P 0.0°,Y 0.0°] | GIMBAL cmd=[0.00°, 0.00°] | att=[-0.0°]
        ↑ Single unified objective: stay level with zero gimbal
```

## Architecture Lessons Learned

### ❌ Bad Design Pattern
```
MPC disabled → desired_acc = 0
             → desired_attitude = level
             → but gimbal still maneuvers
             → FIGHTING!
```

### ✅ Good Design Pattern
```
MPC disabled → desired_acc = 0
             → desired_attitude = level
             → gimbal zeroed out too
             → UNIFIED OBJECTIVE!
```

## What We Can Now Do

With comprehensive debug monitoring, we can:

1. **Verify control consistency** - All layers align on same objective
2. **Detect actuator saturation** - Immediately visible in gimbal/thrust signals
3. **Spot control law bugs** - Attitude errors grow? See it in real-time
4. **Validate new MPC configs** - Watch desired_acc vs reference_acc match
5. **Analyze energy flow** - Track KE/PE changes during flight

## Recommended Next Steps

### For Iteration 13 (MPC Trajectory Tracking)
1. **Re-enable MPC** but with proper weight settings
2. **Watch debug output** for gimbal saturation
3. **Check consistency:** Desired attitude should point where MPC wants thrust
4. **If gimbal saturates:** Either increase gimbal authority or reduce MPC aggressiveness

### For Testing Any New Configuration
1. **Always enable debug output first**
2. **Run 30-second test** to verify basic stability
3. **Check these three signals:**
   - Attitude error → 0? (yes = good)
   - Gimbal saturated? (no = good)  
   - Control consistent? (yes = good)
4. **If any fail:** Analyze debug output before changing code

## Files Modified/Created

### Modified
- `src/simulation/simulator.py` - Added debug tracking, gimbal zeroing
- `src/config.py` - Reduced thrust to 600N, tuned weights
- `run_demo.py` - Removed Unicode in output

### Created
- `src/debug_monitor.py` - Comprehensive debug monitor class
- `DEBUG_FINDINGS.md` - Root cause analysis (this file's content)
- `DEBUG_GUIDE.md` - How to interpret debug output

## Key Insight: "See It Before You Fix It"

The most powerful debugging tool is **complete visibility into ALL control signals**. This one change - comprehensive debug output - revealed the gimbal-attitude fighting issue immediately that would have been invisible with just high-level telemetry.

---
**Status**: ✅ Ready for Iteration 13: MPC Implementation
**Next Focus**: Enable MPC with proper weight tuning, monitor gimbal saturation
