#!/usr/bin/env python3
"""
Test curved trajectory following.
Rocket should ascend to 60km while moving 40km horizontally,
then fly horizontally at the peak.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
from src.models.rocket import Rocket
from src.models.trajectory import CurvedAscentPath
from src.controllers.controller import RocketController

print("=" * 80)
print("CURVED TRAJECTORY TEST: 60km altitude, 40km horizontal distance")
print("=" * 80)
print()

# Create trajectory
trajectory = CurvedAscentPath(peak_altitude=60_000.0, horizontal_distance=40_000.0)
print(f"Trajectory Parameters:")
print(f"  Peak altitude: {trajectory.peak_altitude:,.0f} m")
print(f"  Horizontal distance: {trajectory.horizontal_distance:,.0f} m")
print(f"  Phase 1 (curved ascent): {trajectory.phase1_duration:.1f}s")
print(f"  Phase 2 (horizontal cruise): {trajectory.phase2_duration:.1f}s")
print(f"  Total simulation time: {trajectory.total_time:.1f}s")
print()

# Create rocket and controller
rocket = Rocket()
controller = RocketController()

# Initial state
rocket.state.position = np.array([0.0, 0.0, 0.0])
rocket.state.velocity = np.array([0.0, 0.0, 0.0])
rocket.state.orientation = np.array([0.0, 0.0, 0.0])
rocket.state.angular_velocity = np.zeros(3)

# Simulation data
sim_data = {
    'time': [],
    'position': [],
    'velocity': [],
    'altitude': [],
    'desired_position': [],
    'desired_velocity': [],
    'desired_altitude': [],
    'pitch': [],
    'desired_pitch': [],
    'position_error': [],
    'gimbal_yaw': [],
    'gimbal_pitch': [],
}

dt = 0.01
time = 0.0
phase_peak_reached = False
peak_time = None

print("Simulating...")
print(f"{'Time':>7} {'Alt':>10} {'X-Pos':>10} {'Pitch':>8} {'Error':>10} {'Gimbal-Y':>9} {'Gimbal-P':>9}")
print("-" * 80)

max_time = trajectory.total_time + 30.0  # Continue for 30 seconds after curve ends

while time < max_time:
    # Get desired state
    desired = trajectory.get_desired_state(time)
    
    # Compute control inputs
    throttle, gimbal_yaw, gimbal_pitch = controller.compute_control(
        rocket.state,
        desired,
        dt
    )
    
    rocket.set_control_inputs(throttle, gimbal_yaw, gimbal_pitch)
    
    # Record data
    altitude = rocket.state.position[2]
    desired_alt = desired['position'][2]
    position_error = np.linalg.norm(rocket.state.position[:2] - desired['position'][:2])
    
    sim_data['time'].append(time)
    sim_data['position'].append(rocket.state.position.copy())
    sim_data['velocity'].append(rocket.state.velocity.copy())
    sim_data['altitude'].append(altitude)
    sim_data['desired_position'].append(desired['position'].copy())
    sim_data['desired_altitude'].append(desired_alt)
    sim_data['pitch'].append(np.degrees(rocket.state.orientation[1]))
    sim_data['desired_pitch'].append(np.degrees(desired['orientation'][1]))
    sim_data['position_error'].append(position_error)
    sim_data['gimbal_yaw'].append(gimbal_yaw)
    sim_data['gimbal_pitch'].append(gimbal_pitch)
    
    # Check if peak altitude reached
    if altitude >= 55_000 and not phase_peak_reached:
        phase_peak_reached = True
        peak_time = time
    
    # Print progress
    if int(time * 10) % 10 == 0 and time - dt < int(time * 10) / 10:
        pitch_deg = np.degrees(rocket.state.orientation[1])
        print(f"{time:7.2f}s {altitude:10.0f}m {rocket.state.position[0]:10.0f}m {pitch_deg:8.1f}° {position_error:10.1f}m {gimbal_yaw:9.2f}° {gimbal_pitch:9.2f}°")
    
    # Update rocket
    rocket.update(dt)
    time += dt

print()
print(f"Simulation ended at t={time:.2f}s, altitude={rocket.state.position[2]:.1f}m")
if peak_time is not None:
    print(f"Peak altitude (60km) reached at t={peak_time:.2f}s")
print()

# Analysis
print("=" * 80)
print("ANALYSIS")
print("=" * 80)
print()

times = np.array(sim_data['time'])
positions = np.array(sim_data['position'])
altitudes = np.array(sim_data['altitude'])
desired_altitudes = np.array(sim_data['desired_altitude'])
desired_positions = np.array(sim_data['desired_position'])
pitches = np.array(sim_data['pitch'])
desired_pitches = np.array(sim_data['desired_pitch'])
position_errors = np.array(sim_data['position_error'])

# Maximum altitude reached
max_alt_idx = np.argmax(altitudes)
max_altitude = altitudes[max_alt_idx]
time_at_max_alt = times[max_alt_idx]

print(f"Maximum altitude reached: {max_altitude:,.0f}m at t={time_at_max_alt:.1f}s")
print(f"Target peak altitude: {trajectory.peak_altitude:,.0f}m")
print(f"Altitude error at peak: {max_altitude - trajectory.peak_altitude:,.0f}m")
print()

# Horizontal position at peak
x_at_peak = positions[max_alt_idx, 0]
desired_x_at_peak = desired_positions[max_alt_idx, 0]
print(f"Horizontal X position at peak: {x_at_peak:,.0f}m")
print(f"Target X position at peak: {desired_x_at_peak:,.0f}m")
print(f"X position error at peak: {x_at_peak - desired_x_at_peak:,.0f}m")
print()

# Position error analysis
mean_error = np.mean(position_errors)
max_error = np.max(position_errors)
idx_max_error = np.argmax(position_errors)
time_max_error = times[idx_max_error]

print(f"Mean position error (horizontal): {mean_error:,.0f}m")
print(f"Maximum position error: {max_error:,.0f}m at t={time_max_error:.1f}s")
print()

# Pitch angle analysis
mean_pitch_error = np.mean(np.abs(pitches - desired_pitches))
print(f"Mean pitch angle error: {mean_pitch_error:.2f}°")
print()

# Plot results
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
fig.suptitle('Curved Trajectory Following Test: 60km Peak, 40km Horizontal', fontsize=16, fontweight='bold')

# Altitude vs time
ax = axes[0, 0]
ax.plot(times, altitudes / 1000, 'b-', linewidth=2, label='Actual')
ax.plot(times, desired_altitudes / 1000, 'r--', linewidth=2, label='Desired')
ax.axhline(y=60, color='g', linestyle=':', alpha=0.5, label='Target (60km)')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Altitude (km)', fontweight='bold')
ax.set_title('Altitude vs Time')
ax.legend()
ax.grid(True, alpha=0.3)

# X position vs time
ax = axes[0, 1]
ax.plot(times, positions[:, 0] / 1000, 'b-', linewidth=2, label='Actual')
ax.plot(times, desired_positions[:, 0] / 1000, 'r--', linewidth=2, label='Desired')
ax.axhline(y=40, color='g', linestyle=':', alpha=0.5, label='Target (40km)')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('X Position (km)', fontweight='bold')
ax.set_title('X Position vs Time')
ax.legend()
ax.grid(True, alpha=0.3)

# Pitch angle vs time
ax = axes[0, 2]
ax.plot(times, pitches, 'b-', linewidth=2, label='Actual')
ax.plot(times, desired_pitches, 'r--', linewidth=2, label='Desired')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Pitch Angle (degrees)', fontweight='bold')
ax.set_title('Pitch Angle vs Time')
ax.legend()
ax.grid(True, alpha=0.3)

# 2D trajectory (X-Z plane)
ax = axes[1, 0]
ax.plot(positions[:, 0] / 1000, positions[:, 2] / 1000, 'b-', linewidth=2, label='Actual path')
ax.plot(desired_positions[:, 0] / 1000, desired_positions[:, 2] / 1000, 'r--', linewidth=2, label='Desired path')
ax.plot(positions[0, 0] / 1000, positions[0, 2] / 1000, 'go', markersize=10, label='Start')
ax.plot(positions[max_alt_idx, 0] / 1000, positions[max_alt_idx, 2] / 1000, 'bs', markersize=10, label='Peak (actual)')
ax.plot(desired_positions[max_alt_idx, 0] / 1000, desired_positions[max_alt_idx, 2] / 1000, 'rx', markersize=12, markeredgewidth=2, label='Peak (desired)')
ax.set_xlabel('X Position (km)', fontweight='bold')
ax.set_ylabel('Altitude (km)', fontweight='bold')
ax.set_title('2D Trajectory (X-Z plane)')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

# Position error vs time
ax = axes[1, 1]
ax.plot(times, position_errors / 1000, 'g-', linewidth=2)
ax.fill_between(times, 0, position_errors / 1000, alpha=0.3, color='green')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Position Error (km)', fontweight='bold')
ax.set_title('Horizontal Position Error vs Time')
ax.grid(True, alpha=0.3)

# Gimbal angles vs time
ax = axes[1, 2]
gimbal_yaw = np.array(sim_data['gimbal_yaw'])
gimbal_pitch = np.array(sim_data['gimbal_pitch'])
ax.plot(times, gimbal_yaw, 'r-', linewidth=2, label='Gimbal Yaw')
ax.plot(times, gimbal_pitch, 'b-', linewidth=2, label='Gimbal Pitch')
ax.axhline(y=15, color='k', linestyle=':', alpha=0.5, label='±15° limit')
ax.axhline(y=-15, color='k', linestyle=':', alpha=0.5)
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Gimbal Angle (degrees)', fontweight='bold')
ax.set_title('Gimbal Angles vs Time')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('curved_trajectory_test.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'curved_trajectory_test.png'")
plt.show()
