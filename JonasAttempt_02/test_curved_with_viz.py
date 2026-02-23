#!/usr/bin/env python3
"""
Test curved trajectory with 3D visualization.
Records simulation data, then shows interactive 3D viewer.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from src.models.rocket import Rocket
from src.models.trajectory import CurvedAscentPath
from src.controllers.controller import RocketController
from src.visualization.interactive_plotter import InteractiveRocket3DVisualizer

print("=" * 80)
print("CURVED TRAJECTORY TEST WITH 3D VISUALIZATION")
print("=" * 80)
print()

# Create trajectory
trajectory = CurvedAscentPath(peak_altitude=60_000.0, horizontal_distance=40_000.0)
print(f"Trajectory: 60km altitude peak, 40km horizontal distance")
print(f"Phase 1 duration: {trajectory.phase1_duration:.1f}s (curved ascent)")
print(f"Phase 2 duration: {trajectory.phase2_duration:.1f}s (horizontal cruise)")
print()

# Create rocket and controller
rocket = Rocket()
controller = RocketController()

# Initial state - match the trajectory's starting conditions
# The ballistic trajectory starts at (0,0,0) with high upward velocity
trajectory_start = trajectory.get_desired_state(0.0)
rocket.state.position = trajectory_start['position'].copy()
rocket.state.velocity = trajectory_start['velocity'].copy()
rocket.state.orientation = trajectory_start['orientation'].copy()
rocket.state.angular_velocity = np.zeros(3)

# Simulation data for visualization
telemetry = {
    'time': [],
    'position': [],
    'velocity': [],
    'orientation': [],
    'gimbal_pitch': [],
    'gimbal_yaw': [],
    'thrust': [],
    'desired_position': [],  # Track desired position for visualization
    'desired_velocity': [],
}

# Pre-compute target trajectory (smooth curve, not every timestep)
# Sample the desired trajectory at regular intervals to create a clean path
target_telemetry = {
    'time': [],
    'position': [],
    'velocity': [],
    'orientation': [],
}

# Generate target trajectory samples
num_target_samples = 500
for i in range(num_target_samples):
    t_target = (i / num_target_samples) * trajectory.total_time
    desired = trajectory.get_desired_state(t_target)
    target_telemetry['time'].append(t_target)
    target_telemetry['position'].append(desired['position'].copy())
    target_telemetry['velocity'].append(desired['velocity'].copy())
    target_telemetry['orientation'].append(desired['orientation'].copy())

dt = 0.01
time = 0.0
max_time = trajectory.total_time + 30.0

print("Simulating curved trajectory...")

step_count = 0
while time < max_time:
    # Get desired state
    desired = trajectory.get_desired_state(time)
    
    # Compute control inputs
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(
        rocket.state,
        desired,
        dt
    )
    
    # Record telemetry
    telemetry['time'].append(time)
    telemetry['position'].append(rocket.state.position.copy())
    telemetry['velocity'].append(rocket.state.velocity.copy())
    telemetry['orientation'].append(rocket.state.orientation.copy())
    telemetry['gimbal_pitch'].append(gimbal_pitch)
    telemetry['gimbal_yaw'].append(gimbal_yaw)
    telemetry['thrust'].append(thrust)
    telemetry['desired_position'].append(desired['position'].copy())
    telemetry['desired_velocity'].append(desired['velocity'].copy())
    
    # Apply control inputs
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(dt)
    
    time += dt
    step_count += 1
    
    if step_count % 2000 == 0:
        alt = rocket.state.position[2]
        vel_mag = np.linalg.norm(rocket.state.velocity)
        print(f"  Time: {time:6.2f}s | Altitude: {alt:9.0f}m | Velocity: {vel_mag:8.1f}m/s")

print(f"Simulation complete: {step_count} steps, {time:.2f}s of simulation")
print()

# Convert to numpy arrays for plotter
for key in ['position', 'velocity', 'orientation']:
    telemetry[key] = np.array(telemetry[key])
    target_telemetry[key] = np.array(target_telemetry[key])

telemetry['time'] = np.array(telemetry['time'])
target_telemetry['time'] = np.array(target_telemetry['time'])

# Print summary
max_alt_idx = np.argmax(telemetry['position'][:, 2])
max_altitude = telemetry['position'][max_alt_idx, 2]
time_at_max = telemetry['time'][max_alt_idx]

print("=" * 80)
print("SIMULATION SUMMARY")
print("=" * 80)
print(f"Maximum altitude reached: {max_altitude:,.0f}m at t={time_at_max:.1f}s")
print(f"Target peak altitude: {trajectory.peak_altitude:,.0f}m")
print(f"X position at peak: {telemetry['position'][max_alt_idx, 0]:,.0f}m")
print()

print("Launching 3D visualization...")
print("(Close the plot window to exit)")
print()

# Create and show interactive plotter
plotter = InteractiveRocket3DVisualizer(
    telemetry=telemetry,
    target_telemetry=target_telemetry
)
plotter.show()
