#!/usr/bin/env python3
"""
Debug - trace one full simulation step
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from src.models.rocket import Rocket
from src.models.trajectory import VerticalLaunchPath
from src.controllers.controller import RocketController

# Create objects
rocket = Rocket()
trajectory = VerticalLaunchPath(target_altitude=40_000.0)
controller = RocketController()

# Set initial state
rocket.state.velocity[2] = 5.0

print("Step 0:")
print(f"  Position: {rocket.state.position}")
print(f"  Velocity: {rocket.state.velocity}")

# Get desired state
desired_state = trajectory.get_desired_state(0.0)
print(f"\nDesired state:")
print(f"  Position: {desired_state['position']}")
print(f"  Velocity: {desired_state['velocity']}")

# Compute control
thrust, gimbal_pitch, gimbal_yaw = controller.compute_control(rocket.state, desired_state, 0.01)
print(f"\nControl input:")
print(f"  Thrust: {thrust:.3f}")
print(f"  Gimbal pitch: {np.degrees(gimbal_pitch):.2f}°")
print(f"  Gimbal yaw: {np.degrees(gimbal_yaw):.2f}°")

# Apply control
rocket.set_control_inputs(thrust, gimbal_pitch, gimbal_yaw)

# Take step
rocket.update(0.01)

print(f"\nAfter step:")
print(f"  Position: {rocket.state.position}")
print(f"  Velocity: {rocket.state.velocity}")
print(f"  Z velocity: {rocket.state.velocity[2]:.4f} m/s")

# Take a few more steps
for step in range(1, 500):
    desired_state = trajectory.get_desired_state(step * 0.01)
    thrust, gimbal_pitch, gimbal_yaw = controller.compute_control(rocket.state, desired_state, 0.01)
    rocket.set_control_inputs(thrust, gimbal_pitch, gimbal_yaw)
    rocket.update(0.01)
    
    if step % 50 == 0:
        print(f"Step {step}: Z={rocket.state.position[2]:.1f}m, Vz={rocket.state.velocity[2]:.2f}m/s, Thrust={thrust:.2f}")
        if rocket.state.position[2] < -10:
            print("ERROR: Rocket below ground!")
            break
