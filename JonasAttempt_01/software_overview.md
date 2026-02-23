# Rocket Gimbal Controller - Software Overview

## Project Purpose

This project is a **3D rocket gimbal controller simulation** developed for a control systems coursework project (DHBW - Regelungssysteme). It demonstrates Model Predictive Control (MPC) for trajectory tracking using realistic actuator dynamics, GPS/IMU-like sensor inputs, and 3D visualization.

The system simulates a rocket following complex 3D trajectories using thrust vectoring (gimbal control), with path-based deviation tracking and progress-adaptive trajectory following.

## Key Features

### Control System
- **MPC Controller**: Convex optimization-based Model Predictive Control using CVXPY with OSQP solver
- **Progress-Based Tracking**: Adaptive trajectory following that adjusts to timing deviations (0.2x - 2.0x speed scaling)
- **Path-Based Deviation Metrics**: Cross-track error calculation using closest point on trajectory instead of time-based waypoints
- **15-Step Prediction Horizon**: Looks ahead to optimize trajectory following
- **Constraint Handling**: Maximum acceleration (12 m/s²) and lateral acceleration (2.5 m/s²) limits

### Actuator Models
- **Thrust Actuator**: 
  - First-order lag dynamics (τ = 0.35s)
  - Rate limiting (8000 N/s)
  - Range: 0-2000 N
- **Gimbal Actuator**:
  - Second-order dynamics with velocity and acceleration limits
  - Maximum deflection: ±15°
  - Rate limit: 60°/s
  - Acceleration limit: 300°/s²
- **Body Attitude Dynamics**:
  - First-order rotational response (τ = 0.4s)
  - Rate limit: 70°/s

### Trajectory Options
1. **Helix Trajectory**: Spiral ascent/descent pattern
2. **Upward Curve Trajectory**: Sinusoidal vertical path
3. **Parabolic Cruise Trajectory** (current default):
   - Vertical ascent to 600m peak
   - Level cruise phase
   - Parabolic descent back to ground
   - Smooth transitions using smoothstep interpolation

### Visualization
- **Three-Panel Layout** (22x10 figure):
  - **Left Panel**: Rocket close-up view with synchronized camera control
    - Shows rocket attitude, gimbal deflection, and thrust in detail
    - Thrust vector color-coded by intensity (gold/orange/red)
    - Transparent gimbal deflection cone
    - Coordinate frame and body axis reference
    - Real-time text overlay (pitch, yaw, gimbal, thrust)
    - **Enlarged**: Takes 40% of figure width (width_ratio=1.6)
    - **View Synchronization**: Camera angles automatically match main trajectory view as you drag/rotate
    - **Coordinate Frame**: Uses same rotation convention as main trajectory (elev=30, azim=45 default)
  - **Center Panel**: Main 3D trajectory view with interactive rotation
    - Full trajectory path (green dashed line)
    - Actual flight path trail (orange, downsampled)
    - Rocket body with proper attitude
    - Closest path point marker
    - **Interactive**: Drag to rotate view - closeup automatically follows
    - Adaptive axis scaling based on trajectory extent
  - **Right Panel**: MPC Controller Internals (2×2 grid)
    - **Top-left**: Position error over time (3D error magnitude)
    - **Top-right**: MPC horizon projection (XY view showing next 15 waypoints)
    - **Bottom-left**: MPC costs breakdown (position/velocity/acceleration/jerk/total)
    - **Bottom-right**: Info text (frame number, simulation time)
- **Animation Speed Controls**: 1x, 2x, 4x, 8x, 16x, 32x playback speed
- **Timeline Seekbar**: Frame-accurate seeking with play/pause button
- **Performance Optimizations**:
  - Update interval: Every 2 simulation steps
  - Trail downsampling: Every 2nd point
  - Reduced mesh complexity: 6 points for cylinders/cones
  - Static elements pre-rendered
- **Ground-Hit Detection**: Simulation stops when rocket reaches ground

## System Architecture

```
JonasAttempt_01/
├── run_demo.py              # Main entry point
├── requirements.txt         # Python dependencies
├── README.md               # Quick start guide
├── software_overview.md    # This file
└── src/
    ├── config.py           # Centralized configuration (all parameters)
    ├── models/
    │   ├── trajectory.py   # Trajectory generators (helix, curve, parabolic)
    │   └── actuators.py    # Thrust & gimbal actuator dynamics
    ├── controllers/
    │   ├── mpc.py         # MPC controller with CVXPY
    │   └── pid.py         # Legacy PID helpers (unused)
    ├── simulation/
    │   └── simulator.py   # Main simulation loop & physics integration
    └── visualization/
        └── plotter.py     # 3D animation & plotting
```

## Core Components

### 1. Configuration System (`src/config.py`)
Centralized parameter management using Python dataclasses:
- **PhysicalConfig**: Gravity, mass (50 kg), max thrust (2000 N)
- **ActuatorConfig**: All actuator time constants, rate/acceleration limits
- **MPCConfig**: Horizon steps, cost weights, constraints, progress tracking settings
- **SimulationConfig**: Time step (0.02s), duration (90s), trajectory type selection
- **VisualizationConfig**: Trail length, axis margins, performance settings (update interval, downsampling, mesh quality), layout options (close-up, MPC internals, projection type)
- **DebugConfig**: Progress/MPC debug printout controls

### 2. MPC Controller (`src/controllers/mpc.py`)
Implements constrained Model Predictive Control:
- **Inputs**: Current state (position, velocity) + reference trajectory horizon
- **Outputs**: Dictionary containing:
  - Acceleration command
  - Predicted positions/velocities/accelerations over horizon
  - Reference trajectory over horizon
  - Cost breakdown (position, velocity, acceleration, jerk, total)
  - Solver status
- **Optimization**: Minimizes position error, velocity error, acceleration magnitude, and jerk
- **Constraints**: Maximum total and lateral acceleration limits
- **Solver**: CVXPY with OSQP backend for fast convex optimization

**Cost Function Weights** (tunable in config):
- Position error: 30.0
- Velocity error: 15.0
- Acceleration: 5.0
- Jerk (thrust smoothness): 10.0

### 3. Trajectory Models (`src/models/trajectory.py`)
Three trajectory generators with common interface:
- `get_state(t)`: Returns position, velocity, acceleration at time t
- `time_from_position(pos)`: Estimates trajectory progress from current position (enables progress tracking)

**Parabolic Cruise Trajectory** (current default):
- Phase 1: Vertical ascent (0-1000m over 45s)
- Phase 2: Level cruise at 600m (700m horizontal over 15s)
- Phase 3: Parabolic descent (2000m over 32s)
- Smooth transitions using smoothstep function

### 4. Simulator (`src/simulation/simulator.py`)
Main physics integration and control loop:
- **State**: Position, velocity, thrust, gimbal angles, body attitude
- **Control Flow**:
  1. Get current trajectory reference
  2. Estimate progress along path
  3. Generate future reference horizon (with progress-based scaling)
  4. Call MPC to compute desired acceleration
  5. Convert to thrust magnitude and gimbal deflection
  6. Update actuator states with realistic dynamics
  7. Integrate physics (Newton's laws)
  8. Track closest point on trajectory for deviation metrics
- **Body Attitude Control**: **Aligns with desired velocity direction** (flight path tangent)
  - Rocket "leans into" the trajectory like an aircraft
  - More natural appearance during horizontal flight
  - Gimbal compensates to point thrust in required direction
- **Integration**: Explicit Euler with dt=0.02s
- **History Tracking**: Stores all states + MPC internals for visualization

### 5. Visualization (`src/visualization/plotter.py`)
Interactive 3D animation system with three-panel layout:
- **Left Panel - Rocket Close-Up**:
  - Fixed camera angle (15° elevation, 45° azimuth)
  - Detailed rocket body with gimbal deflection cone
  - Color-coded thrust vector (gold→orange→red by intensity)
  - Coordinate frame and body axis reference lines
  - Real-time status text overlay
- **Center Panel - Main Trajectory View**:
  - Full trajectory path visualization
  - Actual flight trail (downsampled for performance)
  - Dynamic rocket body rendering
  - Adaptive axis scaling (preserves aspect ratios)
- **Right Panel - MPC Internals**:
  - Position error time series (rolling 200-step window)
  - MPC prediction horizon (2D projection showing next 15 waypoints)
  - Cost component breakdown (live visualization of optimization goals)
- **Performance Features**:
  - Update every 2 simulation steps (~2x speedup)
  - Trail downsampling (every 2nd point, ~2-3x rendering speedup)
  - Reduced mesh complexity (8 points vs 12-16, ~30% faster)
  - Interactive speed controls (1x through 32x)
- **Auto-Scaling**: Axes adjust to show entire trajectory without cutting off

## Implementation Details

### Progress-Based Tracking
Traditional time-based trajectory tracking fails when the rocket's timing deviates from the reference (too fast/slow). This implementation uses **progress-based tracking**:

1. Estimate current progress along path using `time_from_position()`
2. Generate future reference states scaled by progress rate (0.2x - 2.0x)
3. MPC optimizes to reach future waypoints regardless of absolute timing
4. Result: Robust path following even with speed deviations

### Path-Based Deviation
Instead of measuring error to time-based waypoint, the system calculates **cross-track error**:
- Find closest point on trajectory to current position
- Measure perpendicular distance to path
- Enables accurate path-following assessment independent of timing

### Gimbal Control Strategy
The rocket uses thrust vectoring with velocity-aligned body attitude:
1. MPC computes desired 3D acceleration
2. **Body attitude** aligns with **desired velocity direction** (trajectory tangent)
   - Rocket tilts to follow the flight path like an aircraft
   - Creates natural-looking horizontal flight during cruise
   - Minimizes need for large gimbal deflections
3. **Gimbal deflection** compensates to point thrust in required direction
   - Calculated in body frame relative to velocity-aligned attitude
   - Provides fine control authority for path corrections
4. Thrust magnitude scaled from vertical acceleration component
5. Actuator dynamics apply realistic rate/acceleration limits

**Benefits**:
- More intuitive visualization (rocket "leans" into turns)
- Reduced gimbal usage during steady flight
- Better separation of path-following (body) and stabilization (gimbal) tasks

### Ground Collision
Simulation automatically terminates when rocket altitude < 1m, preventing unrealistic underground flight.

## Current Configuration (Default)

### Physical Parameters
- Mass: 50 kg
- Max Thrust: 2000 N
- Gravity: 9.81 m/s²

### MPC Tuning
- Horizon: 15 steps (0.3s lookahead)
- Max Acceleration: 12 m/s²
- Max Lateral Acceleration: 2.5 m/s²
- Progress tracking: Enabled (0.2x - 2.0x scaling)

### Trajectory (Parabolic Cruise)
- Peak altitude: 600 m
- Ascent: 1000 m over 45 s
- Cruise: 700 m over 15 s
- Descent: 2000 m over 32 s
- Total duration: ~92 s

### Actuator Limits
- Thrust response: 0.35 s time constant, 8000 N/s rate
- Gimbal: ±15° max, 60°/s rate, 300°/s² acceleration

## Usage

### Running the Demo
```bash
# Create virtual environment (recommended)
python -m venv venv
venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Run simulation
python run_demo.py
```

### Changing Trajectories
Edit `src/config.py`:
```python
trajectory_type: str = "parabolic_cruise"  # or "helix" or "upward_curve"
```

### Tuning MPC
Adjust weights in `MPCConfig` section of `src/config.py`:
- Increase `weight_pos` for tighter path following
- Increase `weight_vel` for smoother velocity tracking
- Increase `weight_accel` to reduce aggressive maneuvers
- Increase `weight_jerk_thrust` for smoother thrust changes

### Debug Output
Enable/disable debug prints in `DebugConfig`:
```python
show_progress: bool = True           # Simulation progress
show_mpc_debug: bool = True          # MPC solver details
```

## Dependencies

- **numpy >= 1.26**: Numerical computations
- **matplotlib >= 3.8**: 3D visualization and plotting
- **cvxpy >= 1.4**: Convex optimization framework for MPC
- **osqp >= 0.6**: Fast QP solver backend

## Gimbal Physics & Body Attitude Dynamics

### Gimbal Torque Model
The gimbal actuator creates reaction torques on the rocket body through thrust vectoring:

**Gimbal Force Calculation** (in body frame):
```
F_gimbal_body = thrust_magnitude * (-gimbal_direction_body)
```
The negative sign is **critical**: gimbal points the thrust, but the reaction torque is opposite.

**Torque Generation**:
```
torque = gimbal_offset_vector × F_gimbal_body
```
Where `gimbal_offset_vector = [0, 0, -3.5m]` (engine is 3.5m behind center of mass)

**Angular Acceleration**:
```
angular_acceleration = torque / moment_of_inertia
I_xy = m * offset² / 3.0  ≈ 204 kg·m²
I_z = m * offset² / 12.0 ≈ 51 kg·m²
```

### Body Attitude Integration
The rocket's attitude (pitch, yaw, roll) is derived from angular velocity integration:
```
dω/dt = gimbal_torque / I + attitude_feedback
attitude += ω * dt
```

**Key Points**:
- Attitude is NOT directly commanded - it emerges from gimbal torques
- Angular velocity accumulates from torques with first-order feedback blending
- Visualization and simulator use identical `_rotation_matrix_from_pitch_yaw()` formula

### Sign Convention Alignment
Recent development discovered multiple layers of sign convention mismatches:

**Issue 1: Gimbal Command Sign Convention**
- MPC outputs gimbal angles in one convention
- Physics engine expected opposite convention for correct torque direction
- **Fix**: Negate gimbal command before passing to actuator: `gimbal_actuator.update(-gimbal_cmd, dt)`

**Issue 2: Gimbal Force Direction**
- Gimbal points thrust in one direction; reaction torque is opposite
- Simple calculation `F = thrust * gimbal_dir` was wrong
- **Fix**: Use `F_body = thrust * (-gimbal_dir_body)` for correct reaction torque

**Issue 3: Visualization Display**
- Plotter needed to show physical gimbal reality, not MPC command
- Requires additional negation in display: `pitch, yaw = g[0], -g[1]`
- **Result**: Visual gimbal direction now matches actual physics

### Known Issues - UNFIXED
- **Exit Code 1 Error**: Simulation crashes on run, likely introduced by gimbal physics changes
  - Status: Incomplete - needs debugging
  - Symptom: `python .\run_demo.py` returns Exit Code 1
  - Possible causes: Syntax error, import error, or runtime exception
  - Impact: Cannot verify gimbal fixes are working correctly

## Recent Changes (February 2026)

### UI Layout Improvements
1. **Enlarged Left Panel**: Increased gridspec width_ratio from 1.0 to 1.6
   - Left pane now takes 40% of figure width
   - Better visibility of rocket attitude and gimbal state
   
2. **Unified Viewing Angles**: 
   - Changed closeup view from `elev=20, azim=45` to `elev=30, azim=45`
   - Now matches main trajectory view for consistent orientation
   
3. **Live View Synchronization**:
   - Added `on_motion` event handler to main trajectory view
   - Closeup camera automatically follows when user rotates main view
   - Angles synced in real-time with threshold of 0.1° to prevent jitter
   - Both panels always show same rotational perspective

### Gimbal Physics Alignment
- Negated gimbal command at actuator input: `-gimbal_cmd`
- Negated gimbal force for torque calculation: `-gimbal_dir_body`
- Updated visualization to show inverted yaw: `pitch, yaw = g[0], -g[1]`
- Enhanced debug output: Shows `att(r,p,y)` and `ang_vel(r,p,y)` in degrees

## Limitations & Extensions

### Current Limitations
- **CRITICAL**: Exit Code 1 error prevents validation of recent gimbal fixes
- Simplified 3-DOF point-mass model (no full 6-DOF rigid body dynamics)
- No wind disturbances or sensor noise
- No aerodynamic forces
- Actuator models are simplified (realistic time constants but idealized dynamics)

### Potential Extensions
1. **Fix Exit Code 1**: Debug and resolve simulation crash
2. **Additional Trajectories**: Orbital (high parabolic arc to apogee), landing approach
3. **Disturbances**: Wind gusts, sensor noise, thrust variations
4. **6-DOF Dynamics**: Full rigid-body rotation with moments of inertia
5. **Aerodynamics**: Drag, lift, stability derivatives
6. **State Estimation**: Kalman filter for noisy sensor fusion
7. **Multiple Rockets**: Formation flight control
8. **Hardware-in-Loop**: Interface with real actuators/sensors

## Development Notes

This is a **teaching/demonstration project** focused on control theory concepts:
- MPC formulation and tuning
- Actuator dynamics and rate limiting
- Trajectory generation and path following
- Real-time visualization

The code prioritizes clarity and modularity over computational efficiency. All parameters are easily accessible in `config.py` for experimentation and tuning exercises.

## Version Information

- **Project**: JonasAttempt_01
- **Status**: UI layout complete with view synchronization; gimbal physics fixes applied but unvalidated (Exit Code 1 error blocks testing)
- **Last Updated**: February 2026
- **Python**: 3.13 (compatible with 3.9+)
- **Author**: Jonas (DHBW Regelungssysteme project)
