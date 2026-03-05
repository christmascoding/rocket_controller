# JonasAttempt_03 - 6-DOF Rocket Control System

This project implements a complete 6-DOF rocket physics simulation, 3-loop cascaded control architecture, and dual-view visualization for a class project.

## Quick Start

1. Create a Python environment and install dependencies:
   - `pip install -r requirements.txt`
2. Run the demo:
   - `python run_demo.py`

## Overview

- **6-DOF Dynamics**: Position, velocity, Euler attitude, and angular rates with aerodynamic lift/drag and CP/CG restoring moments.
- **Control**: 3-loop cascaded (position → desired force → desired orientation → torque/gimbal or aero surface).
- **Visualization**: Left subplot (local locked camera) + Right subplot (global trajectory with target marker).

## Files

- `run_demo.py` — comprehensive launch script
- `src/physics/` — dynamics + aerodynamics
- `src/controllers/` — cascaded controller
- `src/models/` — rocket properties + trajectory
- `src/simulation/` — simulator loop
- `src/visualization/` — Matplotlib 3D visualization
