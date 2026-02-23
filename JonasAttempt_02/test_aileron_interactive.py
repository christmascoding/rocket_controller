#!/usr/bin/env python3
"""
Interactive visualization of aileron gliding test.
Pre-computes the gliding simulation, then shows with playback controls.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from src.models.rocket import Rocket
from src.models.trajectory import VerticalLaunchPath
from src.config import RocketSpecs
from src.visualization.interactive_plotter import InteractiveRocket3DVisualizer

print("=" * 70)
print("AILERON GLIDE TEST - INTERACTIVE VISUALIZATION")
print("=" * 70)
print()

# Create rocket
rocket = Rocket()

# Initial state: 10km altitude, 70° angle to the right, no thrust
velocity_magnitude = 100.0
angle_rad = np.radians(70.0)

rocket.state.position = np.array([0.0, 0.0, 10_000.0])
rocket.state.velocity = np.array([0.0, velocity_magnitude * np.cos(angle_rad), velocity_magnitude * np.sin(angle_rad)])
rocket.state.orientation = np.array([0.0, angle_rad, 0.0])  # 70° pitch
rocket.state.angular_velocity = np.zeros(3)

# No thrust input
rocket.set_control_inputs(0.0, 0.0, 0.0)

# Simulate
print("Computing gliding trajectory...")
dt = 0.01
max_time = 30.0
t = 0.0

telemetry = {
    'time': [],
    'position': [],
    'velocity': [],
    'altitude': [],
    'velocity_magnitude': [],
    'thrust': [],
    'gimbal_pitch': [],
    'gimbal_yaw': [],
    'orientation': [],  # Add actual rocket orientation
}

while t < max_time and rocket.state.position[2] > 0:
    telemetry['time'].append(t)
    telemetry['position'].append(rocket.state.position.copy())
    telemetry['velocity'].append(rocket.state.velocity.copy())
    telemetry['altitude'].append(rocket.state.position[2])
    telemetry['velocity_magnitude'].append(np.linalg.norm(rocket.state.velocity))
    telemetry['thrust'].append(rocket.thrust)
    telemetry['gimbal_pitch'].append(rocket.gimbal_pitch)
    telemetry['gimbal_yaw'].append(rocket.gimbal_yaw)
    telemetry['orientation'].append(rocket.state.orientation.copy())  # Store actual orientation
    
    rocket.update(dt)
    t += dt
    
    if int(t) % 5 == 0 and t - dt < int(t):
        print(f"  t={t:.1f}s | alt={rocket.state.position[2]:.0f}m | vel={np.linalg.norm(rocket.state.velocity):.1f}m/s")

print(f"Done. Computed {len(telemetry['time'])} frames in {t:.1f}s of simulation")
print()

# Create a dummy trajectory for compatibility
class DummyTrajectory:
    def get_desired_state(self, t):
        return {'position': np.array([0, 0, 10000]), 'velocity': np.array([0, 0, 0])}

print("Launching interactive viewer...")
visualizer = InteractiveRocket3DVisualizer(telemetry)
print("Controls:")
print("  - Play/Pause: Start/stop playback")
print("  - Speed buttons: Control playback speed")
print("  - Seekbar: Jump to any point in the flight")
print()
visualizer.show()
