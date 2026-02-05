# Rocket Gimbal Controller (JonasAttempt_01)

This project provides a basic, modular Python codebase for a 3D rocket guidance and gimbal control demo. It simulates GPS/IMU-like state signals, applies an MPC controller, and visualizes target vs. actual trajectories plus gimbal direction.

## Quick Start

1. Create a virtual environment and install dependencies:
   - `pip install -r requirements.txt`
2. Run the demo:
   - `python run_demo.py`

## Structure

- `src/config.py` — Physical constants, actuator limits, controller gains
- `src/models/trajectory.py` — Target trajectory generation
- `src/models/actuators.py` — Thrust and gimbal actuator dynamics
- `src/controllers/pid.py` — Generic PID helpers
- `src/controllers/mpc.py` — MPC controller (constraints and horizon-based tracking)
- `src/simulation/simulator.py` — Main simulation loop
- `src/visualization/plotter.py` — 3D plotting/animation

## Notes

- This is a simplified model intended for control-systems coursework. It uses realistic actuator limits and time constants, but it is not a full 6-DOF rigid-body simulation.
- Noise and wind models are intentionally left as extension points.
