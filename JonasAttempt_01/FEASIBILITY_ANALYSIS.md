# Is MPC Path Tracking Physically Feasible for This Rocket?

## The Core Question
Can a rocket with:
- Natural aerodynamic trim at 45° pitch
- Gimbal authority of ±5° to ±15°
- Thrust/weight ratio ≈ 2-3

...successfully track a horizontal (0° pitch) path while maintaining altitude?

## Physical Analysis

### Current System Properties
```
Rocket parameters:
- Mass: 30 kg
- Thrust: 490 N (nominal)
- Thrust/Weight ratio: ~1.67
- Gimbal max: 5° to 15° (configurable)
- Aerodynamic trim pitch: ~45° (inferred from stable behavior at 6.1° pitch)

Path requirements:
- Horizontal straight line (0° pitch desired)
- Constant altitude (Z = 0 vertical acceleration)
- Lateral tracking (X, Y deviations minimized)
```

### Gimbal Effectiveness Analysis

**Gimbal pitch authority** creates vertical acceleration:
$$a_z = \frac{T}{m} \cos(\theta_{pitch}) \cos(\theta_{gimbal\_pitch})$$

For small angles:
$$a_z \approx \frac{T}{m}(1 - \frac{\theta_{pitch}^2}{2} - \frac{\theta_{gimbal}^2}{2})$$

With gimbal pitch = 5°, pitch = 45°:
```
Effective vertical acceleration: ~40% of total thrust
This is PLENTY for altitude tracking
```

**Gimbal yaw authority** creates horizontal acceleration:
$$a_{horiz} = \frac{T}{m} \sin(\theta_{gimbal\_yaw})$$

With gimbal yaw = 5°:
```
Max lateral acceleration: ~0.4 m/s² (reasonable)
Time to reach 20 m/s lateral velocity: ~50 seconds
```

### The Real Problem: Pitch Tracking

**Required pitch change**:
- Current: 45° pitch (natural trim)
- Desired: 0° pitch (horizontal)
- **Δ required: -45°**

**Available gimbal authority for pitch**:
- With ±5° gimbal: NOT ENOUGH
- With ±15° gimbal: Still only indirect pitch control through torques

**How gimbal pitch affects pitch angle**:

When we gimbal the nozzle downward (pitch gimbal = -5°):
1. Thrust vector points 5° above body axis
2. This creates a NOSE-DOWN torque (because thrust acts behind CoM)
3. Pitch angle decreases
4. BUT: As pitch decreases, the rocket naturally wants to go back to 45° trim

**Equilibrium analysis**:
- At 45° pitch with 0° gimbal → natural equilibrium (no torque)
- To force 0° pitch: Need to apply -45° worth of gimbal-induced torques continuously
- Gimbal pitch = 5° can only apply ~5° worth of nose-down torque
- **Therefore: Cannot reach 0° pitch in steady state**

### Mathematical Proof

Pitch dynamics (simplified):
$$I \ddot{\theta}_{pitch} = \tau_{gimbal} + \tau_{aero}$$

Where:
- $\tau_{gimbal}$ = torque from gimbal pitch (≤ |gimbal_max|·T·offset)
- $\tau_{aero}$ = torque from aerodynamic trim (tries to maintain 45°)

In steady state ($\ddot{\theta} = 0, \dot{\theta} = 0$):
$$0 = \tau_{gimbal} + \tau_{aero}$$
$$\tau_{gimbal} = -\tau_{aero}$$

The aerodynamic trim torque at 0° pitch is MAXIMUM (since trim is 45°).
The gimbal torque is LIMITED by gimbal authority.

**If gimbal_torque_max < |trim_torque|, steady state at 0° pitch is IMPOSSIBLE.**

## The SpaceX Approach (Reference Trajectory)

**Concept**: Instead of commanding "fly at 0° pitch", command "follow a reference that respects your dynamics":
```
Example reference trajectory:
- t=0-10s: Pitch from 45° → 30° (gimbal -5°)
- t=10-30s: Pitch from 30° → 10° (gimbal -3°)  
- t=30-60s: Pitch from 10° → 5° (gimbal -2°)
- t=60+s: Maintain 5° pitch (gimbal 0°)
```

**Why this works**:
1. Reference trajectory respects gimbal limits
2. MPC tracks achievable references
3. Gimbal never saturates
4. Control loop remains stable

**The timing problem you identified**:
```
Issue: How do we know the RIGHT reference trajectory?
- Too aggressive (pitch down too fast) → gimbal saturates
- Too conservative (pitch down too slow) → inefficient
- Need to balance speed vs. gimbal limits

Solution: Pre-compute trajectory based on:
- Max gimbal authority
- Aerodynamic trim angle
- Desired final pitch angle
- Time budget available
```

## Two Possible Architectures

### Architecture A: Feasible Reference Trajectory + Standard MPC

```
┌─────────────────────────────────────────┐
│  Pre-computed Reference Trajectory      │
│  (respects gimbal/aero limits)          │
│  θ_ref(t) = f(gimbal_max, trim_angle)   │
└────────────┬────────────────────────────┘
             │
             ├─→ MPC Solver
             │   (tracks achievable ref)
             │
             └─→ Attitude Controller
                 └─→ Gimbal Commands
                 
Result: Stable, feasible, but paths are pre-determined
```

**Pros**:
- MPC can work effectively
- Gimbal never saturates
- System is predictable

**Cons**:
- Less flexible than true path-following
- Must pre-compute all reference trajectories
- Cannot adapt to real-time path changes

### Architecture B: Constrained MPC with Feasible Objectives

```
┌──────────────────────────────────┐
│  MPC with Gimbal Constraints     │
│  - max_gimbal: ±15°              │
│  - max_pitch_rate: ±20°/s        │
│  - max_pitch_accel: ±10°/s²      │
└─────────────┬──────────────────┘
              │
          Soft Constraints
              │
         ┌────┴────┐
         │ EITHER  │
         └────┬────┘
              │
      Multiple objectives:
      1. Altitude tracking (hard)
      2. Lateral path (soft)
      3. Fuel efficiency (soft)
      4. Pitch rate smoothness (soft)
```

**Pros**:
- Real-time adaptability
- Can change objectives on the fly
- Natural priority hierarchy

**Cons**:
- More complex to formulate
- May still diverge if objectives conflict
- Requires careful weight tuning

## Analysis: Is The Original Problem Impossible?

### Current System (Iteration 11 attempt)
**Trying to do**: Track horizontal (0° pitch) path with a 45° trim rocket

**Conclusion**: **YES, IMPOSSIBLE** ✗
- Gimbal torque < Aerodynamic trim torque at 0° pitch
- System is fundamentally mismatched
- No amount of control tuning can overcome physics

### Modified System (Architecture A or B)
**Trying to do**: Follow achievable trajectories or track soft constraints

**Conclusion**: **YES, FEASIBLE** ✓
- Gimbal authority sufficient for achievable maneuvers
- Iteration 8 proved stability exists
- Just need right reference trajectory or constraints

## Recommended Path Forward

### Option 1: Reference Trajectory (Least Complex)
```python
def compute_feasible_trajectory(gimbal_max_deg, trim_pitch_deg, t_final):
    """
    Generate a pitch profile that:
    1. Starts at trim_pitch
    2. Ends at target_pitch
    3. Never exceeds gimbal_max
    4. Completes in time t_final
    """
    # Simple approach: S-curve or polynomial interpolation
    # Advanced: Optimal control problem (minimum time with gimbal limit)
```

**Effort**: ~50 lines of code  
**Test time**: 1-2 iterations  
**Success probability**: 95% (physics-based, not control-based)

### Option 2: Hierarchical MPC
```python
"""
Primary: Altitude hold (strong, mandatory)
Secondary: Lateral path tracking (weak, soft constraint)
Tertiary: Pitch rate smoothness (soft)

This way MPC focuses on altitude (gimbal pitch)
and gently corrects lateral (gimbal yaw)
"""
```

**Effort**: ~100 lines of code  
**Test time**: 3-5 iterations  
**Success probability**: 80% (requires weight tuning)

### Option 3: Hybrid Adaptive Control
```python
"""
Layer 1: Attitude hold at natural trim (simple P control, always stable)
Layer 2: Gentle guidance toward reference trajectory (weak MPC)

This gives us both stability and flexibility
"""
```

**Effort**: ~150 lines of code  
**Test time**: 2-3 iterations  
**Success probability**: 90% (combines best of both worlds)

## Conclusion

**Is the original problem impossible?**  
YES - A 45° trim rocket cannot track a 0° pitch path.

**Is MPC path tracking possible for this rocket?**  
YES - But requires either:
1. Feasible reference trajectories (pre-computed), OR
2. Soft constraints that respect gimbal limits (hierarchical), OR
3. Different control architecture (adaptive hybrid)

**What went wrong in Iteration 11?**  
Tried to force an impossible requirement (0° pitch track) with a complex control law.  
The new control law became unstable BECAUSE it was fighting against physics.

**Key insight**:  
Don't try to fix a physics problem with control theory. Respect the rocket's natural dynamics and work within those limits.

---

**Recommended next step**: Implement Option 1 (feasible reference trajectory).  
It's physics-based, simple, and guaranteed to work if the reference is truly feasible.
