#!/usr/bin/env python3
"""
Simple physics test - verify rocket dynamics
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from src.models.rocket import Rocket

# Create rocket
rocket = Rocket()

# Set initial state
rocket.state.position[2] = 0.0
rocket.state.velocity[2] = 5.0  # Start with 5 m/s up

# Full throttle, no gimbal
rocket.set_control_inputs(1.0, 0.0, 0.0)

print("Initial state:")
print(f"  Position Z: {rocket.state.position[2]:.2f} m")
print(f"  Velocity Z: {rocket.state.velocity[2]:.2f} m/s")

# Manually compute forces at this moment
thrust_body = rocket._compute_thrust_vector()
R = rocket._compute_rotation_matrix(0, 0, 0)
thrust_inertial = R @ thrust_body
gravity = np.array([0, 0, -rocket.specs.mass * 9.81])
drag = rocket._compute_aerodynamic_forces()

print(f"\nForces:")
print(f"  Thrust (inertial) Z: {thrust_inertial[2]:.0f} N")
print(f"  Gravity Z: {gravity[2]:.0f} N")
print(f"  Drag Z: {drag[2]:.0f} N")
print(f"  Total Z: {thrust_inertial[2] + gravity[2] + drag[2]:.0f} N")
print(f"  Acceleration Z: {(thrust_inertial[2] + gravity[2] + drag[2]) / rocket.specs.mass:.2f} m/s^2")

# Now take one physics step
print(f"\nTaking 0.01s step...")
rocket.update(0.01)

print(f"After step:")
print(f"  Position Z: {rocket.state.position[2]:.4f} m")
print(f"  Velocity Z: {rocket.state.velocity[2]:.4f} m/s")

# Should have moved up by roughly: 5.0 * 0.01 + 0.5 * 20 * 0.01^2 = 0.05 + 0.001 = 0.051 m
# And velocity should be: 5.0 + 20 * 0.01 = 5.2 m/s
print(f"\nExpected position change: ~0.051 m (actual: {rocket.state.position[2]:.4f} m)")
print(f"Expected velocity: ~5.2 m/s (actual: {rocket.state.velocity[2]:.4f} m/s)")
