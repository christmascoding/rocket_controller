#!/usr/bin/env python3
"""
Test aileron functionality with rocket pointing downward at 45°.
Start at 10km altitude, pointing downward at 45°, with no thrust.
Observe how ailerons stabilize or steer the rocket.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
from src.models.rocket import Rocket
from src.config import RocketSpecs

print("=" * 70)
print("AILERON TEST: Rocket pointing downward at 45°")
print("=" * 70)
print()

# Create rocket
rocket = Rocket()

# Initial state:
# - Position: 10km altitude
# - Pointing downward at 45° angle
# - Initial downward velocity (falling)
# - No thrust input
print("Initial Conditions:")
print(f"  Altitude: 10,000 m")
print(f"  Orientation: 45° pitch downward")
print(f"  Velocity: ~50 m/s downward")
print()

# Pointing downward at 45° means pitch angle = -45° (negative = downward)
pitch_angle = np.radians(-45.0)

# Initial velocity: straight down at 50 m/s
rocket.state.position = np.array([0.0, 0.0, 10_000.0])
rocket.state.velocity = np.array([0.0, 0.0, -50.0])  # Negative = downward
rocket.state.orientation = np.array([0.0, pitch_angle, 0.0])  # -45° pitch
rocket.state.angular_velocity = np.zeros(3)

# No thrust input
rocket.set_control_inputs(0.0, 0.0, 0.0)

# Simulate for 20 seconds or until impact
simulation_data = {
    'time': [],
    'position': [],
    'velocity': [],
    'altitude': [],
    'velocity_magnitude': [],
    'pitch': [],
    'y_position': [],
}

dt = 0.01
max_time = 20.0
t = 0.0

print("Simulating...")
while t < max_time and rocket.state.position[2] > 0:
    # Record data
    simulation_data['time'].append(t)
    simulation_data['position'].append(rocket.state.position.copy())
    simulation_data['velocity'].append(rocket.state.velocity.copy())
    simulation_data['altitude'].append(rocket.state.position[2])
    simulation_data['velocity_magnitude'].append(np.linalg.norm(rocket.state.velocity))
    simulation_data['pitch'].append(np.degrees(rocket.state.orientation[1]))
    simulation_data['y_position'].append(rocket.state.position[1])
    
    # Update rocket
    rocket.update(dt)
    t += dt
    
    # Print progress
    if int(t) % 2 == 0 and t - dt < int(t):
        alt = rocket.state.position[2]
        vel = np.linalg.norm(rocket.state.velocity)
        pitch = np.degrees(rocket.state.orientation[1])
        print(f"  Time: {t:6.2f}s | Alt: {alt:8.1f}m | Vel: {vel:7.1f}m/s | Pitch: {pitch:7.1f}°")

print()
print(f"Simulation ended at t={t:.2f}s, altitude={rocket.state.position[2]:.1f}m")
print()

# Analysis
print("=" * 70)
print("ANALYSIS")
print("=" * 70)
print()

times = np.array(simulation_data['time'])
positions = np.array(simulation_data['position'])
velocities = np.array(simulation_data['velocity'])
altitudes = np.array(simulation_data['altitude'])
pitches = np.array(simulation_data['pitch'])
vel_mags = np.array(simulation_data['velocity_magnitude'])
y_positions = np.array(simulation_data['y_position'])

print(f"Initial pitch angle: {pitches[0]:.1f}° (downward)")
print(f"Final pitch angle: {pitches[-1]:.1f}°")
print(f"Pitch change: {pitches[-1] - pitches[0]:.1f}°")
print()

# Find minimum altitude (if rocket oscillates)
min_alt_idx = np.argmin(altitudes)
min_alt = altitudes[min_alt_idx]
time_at_min_alt = times[min_alt_idx]

print(f"Minimum altitude reached: {min_alt:.1f}m at t={time_at_min_alt:.1f}s")
print()

# Displacement
start_pos = positions[0]
end_pos = positions[-1]
displacement = end_pos - start_pos
horizontal_displacement = np.sqrt(displacement[0]**2 + displacement[1]**2)

print(f"Horizontal displacement: {horizontal_displacement:.1f}m")
print(f"  - X direction: {displacement[0]:.1f}m")
print(f"  - Y direction: {displacement[1]:.1f}m")
print()

# Final velocity
final_vel = velocities[-1]
final_vel_mag = vel_mags[-1]
print(f"Final velocity: {final_vel_mag:.1f}m/s")
print(f"  - vx: {final_vel[0]:.1f}m/s")
print(f"  - vy: {final_vel[1]:.1f}m/s")
print(f"  - vz: {final_vel[2]:.1f}m/s (downward)")
print()

# Plot results
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Aileron Test: Rocket Pointing Downward at 45°', fontsize=16, fontweight='bold')

# Altitude vs time
ax = axes[0, 0]
ax.plot(times, altitudes, 'b-', linewidth=2)
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Altitude (m)', fontweight='bold')
ax.set_title('Altitude vs Time')
ax.grid(True, alpha=0.3)

# Pitch angle vs time
ax = axes[0, 1]
ax.plot(times, pitches, 'r-', linewidth=2)
ax.axhline(y=-45, color='g', linestyle='--', alpha=0.5, label='Initial pitch (-45°)')
ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Horizontal (0°)')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Pitch Angle (degrees)', fontweight='bold')
ax.set_title('Pitch Angle vs Time (Should show stabilization/correction)')
ax.legend()
ax.grid(True, alpha=0.3)

# Trajectory in XY plane
ax = axes[1, 0]
ax.plot(positions[:, 0], positions[:, 1], 'g-', linewidth=2, label='Trajectory')
ax.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='Start')
ax.plot(positions[-1, 0], positions[-1, 1], 'rx', markersize=12, markeredgewidth=2, label='End')
ax.set_xlabel('X position (m)', fontweight='bold')
ax.set_ylabel('Y position (m)', fontweight='bold')
ax.set_title('Trajectory in XY plane')
ax.legend()
ax.grid(True, alpha=0.3)
ax.axis('equal')

# Altitude vs Y position (side view of falling path)
ax = axes[1, 1]
ax.plot(y_positions, altitudes, 'b-', linewidth=2)
ax.plot(y_positions[0], altitudes[0], 'go', markersize=10, label='Start')
ax.plot(y_positions[-1], altitudes[-1], 'rx', markersize=12, markeredgewidth=2, label='End')
ax.set_xlabel('Y position (m)', fontweight='bold')
ax.set_ylabel('Altitude (m)', fontweight='bold')
ax.set_title('Altitude vs Y position (falling path)')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('aileron_downward_test.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'aileron_downward_test.png'")
plt.show()
