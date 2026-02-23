# Rocket Control System Simulation

A complete 6-DOF (six degrees of freedom) rocket physics simulation with gimbaled engine control and real-time 3D visualization.

## Features

### Physics Model
- **6-DOF Dynamics**: Full translational and rotational equations of motion
- **Gimbaled Engine**: Rocket engine can gimbal up to ±15° in pitch and yaw
- **Realistic Properties**:
  - Rocket diameter: 70cm
  - Rocket length: 5.825m
  - Total mass: 10 tonnes
  - Maximum thrust: 300kN
  - Center of mass: 2.8m above engine gimbal point
- **Environmental Forces**:
  - Gravitational acceleration: 9.81 m/s²
  - Aerodynamic drag: Modeled based on velocity and reference area
  - Air density: Sea level (1.225 kg/m³)

### Control System
- **Automatic Path Following Controller**: Uses PID control to follow desired trajectories
- **Thrust Control**: Modulates engine thrust to maintain altitude and velocity targets
- **Attitude Control**: Uses gimbal angles to control rocket pitch and yaw
- **Angular Velocity Damping**: Prevents excessive rotation

### Trajectory Paths

Three pre-built trajectory paths are available:

1. **Vertical Launch** (Default)
   - Launches straight up to 40km altitude
   - Zero horizontal displacement
   - Complete vertical orientation maintained
   - Time to apogee: ~90 seconds

2. **Spiral Launch**
   - Ascends in a spiral pattern
   - Reaches 40km with 3 complete spiral rotations
   - Combines vertical and horizontal motion
   - Challenging control scenario

3. **Parabolic Launch**
   - Classic ballistic arc trajectory
   - Target altitude: 10km
   - Horizontal range: 5km
   - Demonstrates parabolic path following

### Visualization

Real-time 3D visualization with two synchronized plots:

**Left Plot (Rocket Detail View)**
- Zoomed view of rocket with center of gravity at origin
- Rocket body shown as blue line
- Nose cone rendered as cone structure
- Gimbal thrust vector visualization:
  - Green when thrust is low (0%)
  - Red when thrust is high (100%)
  - Length varies from 1m to 10m based on thrust
  - Automatically adjusts for gimbal angles
- Coordinate axes showing rocket orientation (XYZ)
- Gimbal angle and thrust information displayed

**Right Plot (Trajectory View)**
- Full trajectory history from launch
- Launch point marked in green
- Current position marked in red star
- Automatically scales to show entire flight path
- Same rotation synchronized with rocket view

**Common Features**
- Automatic 360° rotation for all-angle view
- Real-time updates at ~30 Hz
- Displays simulation time, altitude, and velocity

## Project Structure

```
JonasAttempt_02/
├── src/
│   ├── __init__.py
│   ├── config.py              # All rocket and simulation parameters
│   ├── simulation.py          # Main simulation engine
│   ├── models/
│   │   ├── __init__.py
│   │   ├── rocket.py          # 6-DOF rocket physics
│   │   └── trajectory.py      # Trajectory path definitions
│   ├── controllers/
│   │   ├── __init__.py
│   │   └── controller.py      # Path following controller
│   └── visualization/
│       ├── __init__.py
│       └── plotter.py         # 3D visualization
├── run_demo.py                # Main launch script
└── requirements.txt           # Python dependencies
```

## Installation

### Prerequisites
- Python 3.7+
- pip package manager

### Setup

1. Navigate to the JonasAttempt_02 directory:
```bash
cd JonasAttempt_02
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Running the Simulation

### Default (Vertical Launch)
```bash
python run_demo.py
```

This runs a vertical launch to 40km altitude. The simulation will:
1. Display initial parameters in the console
2. Show real-time progress every second
3. Open a visualization window with dual 3D plots
4. Display final results when complete

### Changing Trajectory

Edit `run_demo.py` to uncomment the desired trajectory:

```python
# For spiral launch:
telemetry = run_spiral_launch()

# For parabolic launch:
telemetry = run_parabolic_launch()
```

## Configuration

All simulation parameters can be modified in `src/config.py`:

### Rocket Specifications
- `RocketSpecs`: Physical properties (mass, thrust, dimensions)
- `RocketSpecs.gimbal_max_angle`: Maximum gimbal deflection
- `RocketSpecs.Ixx, Iyy, Izz`: Moments of inertia

### Simulation Parameters
- `SimulationParams.dt`: Time step (default 0.01s)
- `SimulationParams.gravity`: Gravitational acceleration
- `SimulationParams.air_density`: Atmospheric density
- `SimulationParams.max_simulation_time`: Maximum runtime

### Controller Tuning
Edit `src/controllers/controller.py` to adjust PID gains:
```python
self.altitude_controller = PIDController(kp=0.05, ki=0.01, kd=0.1, ...)
self.pitch_controller = PIDController(kp=0.3, ki=0.02, kd=0.05, ...)
self.yaw_controller = PIDController(kp=0.3, ki=0.02, kd=0.05, ...)
```

### Visualization Parameters
- `VisualizationParams.thrust_line_min_length`: Length at 0% thrust
- `VisualizationParams.thrust_line_max_length`: Length at 100% thrust
- `VisualizationParams.rotation_speed`: Plot rotation speed (degrees/second)

## Understanding the Output

### Console Output
```
Time: 100.00s | Altitude: 45000.0m | Velocity: 50.5m/s
```
- **Time**: Elapsed simulation time in seconds
- **Altitude**: Height above launch point in meters
- **Velocity**: Magnitude of velocity vector in m/s

### Visualization
- **Blue line**: Rocket body
- **Green→Red line**: Gimbal thrust vector (color indicates thrust level)
- **Black star**: Center of gravity (always at plot origin)
- **Red star**: Current position in trajectory view
- **Green line**: Trajectory history

## Physics Details

### Coordinate System
- X-axis: Horizontal (right)
- Y-axis: Horizontal (forward)
- Z-axis: Vertical (up)
- Launch point: Origin (0, 0, 0)

### Rocket Orientation (Euler Angles)
- Roll (α₁): Rotation about X-axis
- Pitch (α₂): Rotation about Y-axis  
- Yaw (α₃): Rotation about Z-axis
- Convention: ZYX rotation order

### Control Inputs
1. **Thrust** (0-100%): Modulates engine output
2. **Gimbal Pitch** (±15°): Pitch engine deflection
3. **Gimbal Yaw** (±15°): Yaw engine deflection

### Forces and Torques
- **Thrust Force**: From gimbaled engine (up to 300kN)
- **Gravitational Force**: Constant downward acceleration
- **Aerodynamic Drag**: Proportional to velocity squared
- **Control Torques**: Generated by gimbal offset from center of mass

## Extending the System

### Adding New Trajectories
1. Create a new class inheriting from `TrajectoryPath` in `src/models/trajectory.py`
2. Implement `get_desired_state(time)` method
3. Create a launch function in `run_demo.py`

### Improving the Controller
1. Tune PID gains in `src/controllers/controller.py`
2. Add additional control laws (e.g., feedforward control)
3. Implement adaptive control strategies

### Enhancing Visualization
1. Modify `RocketVisualizer3D` in `src/visualization/plotter.py`
2. Add telemetry plots (thrust history, gimbal angles, etc.)
3. Implement data export to CSV

## Physics Engine Details

The simulation uses 4th-order Runge-Kutta integration for numerical stability and accuracy:

1. **State Vector**: [position, velocity, orientation, angular_velocity]
2. **Dynamics**: Full 6-DOF rigid body equations
3. **Gimbal Effect**: Engine deflection creates moment arm for control torques
4. **Rotation Matrix**: ZYX Euler angle convention for attitude representation

## Troubleshooting

### Simulation runs but visualization doesn't appear
- Check matplotlib installation: `pip install matplotlib --upgrade`
- Try running in a terminal with display support

### Rocket behaves erratically
- Check PID controller gains in `controller.py`
- Reduce time step `dt` in `config.py` for more accuracy
- Verify rocket moment of inertia calculations

### Simulation is too slow
- Disable visualization by modifying `run_demo.py`:
  ```python
  sim = RocketSimulation(trajectory, visualize=False)
  ```
- Reduce `plot_update_hz` in `config.py`
- Increase time step `dt` (trades accuracy for speed)

## Future Enhancements

- [ ] Multiple rocket stages
- [ ] Fuel consumption modeling
- [ ] Atmosphere density variation with altitude
- [ ] Wind disturbances
- [ ] Advanced control algorithms (MPC, LQR)
- [ ] Real-time telemetry export
- [ ] Trajectory optimization
- [ ] Monte Carlo uncertainty analysis

## References

- 6-DOF Rigid Body Dynamics
- Control Systems Theory (PID Control)
- Rocket Propulsion Principles
- Trajectory Mechanics

## License

This project is part of the DHBW Regelungssysteme (Control Systems) course.

---

**Author**: Generated for Control Systems Course Project  
**Date**: 2026  
**Status**: Fully Functional
