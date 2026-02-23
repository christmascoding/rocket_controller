#!/usr/bin/env python3
"""
Quick test of new control theory-based controller.
"""

import sys
sys.path.insert(0, '.')

from src.models.rocket import RocketState, Rocket
from src.models.trajectory import CurvedAscentPath
from src.controllers.controller import RocketController
import numpy as np

# Create trajectory
trajectory = CurvedAscentPath(
    peak_altitude=60000.0,
    horizontal_distance=40000.0
)

# Get initial states
initial_state = trajectory.get_desired_state(0.0)
rocket = Rocket()
rocket.state = RocketState(
    position=initial_state['position'].copy(),
    velocity=initial_state['velocity'].copy(),
    orientation=initial_state['orientation'].copy(),
    angular_velocity=np.array([0.0, 0.0, 0.0])
)

controller = RocketController()

print("="*100)
print("CONTROL THEORY TEST - TRAJECTORY TRACKING")
print("="*100)
print()
print(f"{'Time(s)':>8} | {'Actual Z(km)':>12} | {'Desired Z(km)':>12} | {'Error(km)':>10} | "
      f"{'Gimbal P(°)':>12} | {'Gimbal Y(°)':>12} | {'Thrust':>8}")
print("-"*100)

dt = 0.01
time = 0.0
max_time = 120.0

step = 0
while time < max_time:
    desired = trajectory.get_desired_state(time)
    
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(
        rocket.state,
        desired,
        dt
    )
    
    # Print every 10 seconds
    if step % (int(10.0 / dt)) == 0:
        error_z = (rocket.state.position[2] - desired['position'][2]) / 1000.0
        print(f"{time:8.1f} | {rocket.state.position[2]/1000:12.2f} | {desired['position'][2]/1000:12.2f} | "
              f"{error_z:10.3f} | {np.degrees(gimbal_pitch):12.2f} | {np.degrees(gimbal_yaw):12.2f} | {thrust:8.3f}")
    
    # Apply control
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(dt)
    
    time += dt
    step += 1

print()
print("="*100)
print("FINAL STATE")
print("="*100)
print(f"Actual altitude: {rocket.state.position[2]/1000:.2f}km")
print(f"Target altitude: 60.00km")
print(f"Error: {(rocket.state.position[2] - 60000.0)/1000:.2f}km")
