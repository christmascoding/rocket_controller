#!/usr/bin/env python3
"""
Test aileron/lift functionality.
Start rocket 10km in sky at 70° angle with zero thrust.
Should glide towards the velocity direction (natural stability).
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
from src.models.rocket import Rocket
from src.config import RocketSpecs

print("=" * 70)
print("AILERON TEST: Gliding behavior")
print("=" * 70)
print()

# Create rocket
rocket = Rocket()

# Initial state:
# - Position: 10km altitude
# - Velocity: 100 m/s at 70° angle to the right (positive Y)
# - Orientation: 70° pitch angle
# - No thrust input
print("Initial Conditions:")
print(f"  Altitude: 10,000 m")
print(f"  Velocity magnitude: 100 m/s at 70° angle")
print()

# Calculate velocity components for 70° angle
velocity_magnitude = 100.0
angle_deg = 70.0
angle_rad = np.radians(angle_deg)

# At 70° to the right means:
# Mostly horizontal with some vertical
vx = 0.0
vy = velocity_magnitude * np.cos(angle_rad)  # ~34 m/s horizontal to the right
vz = velocity_magnitude * np.sin(angle_rad)  # ~94 m/s vertical

print(f"  Velocity components: vx={vx:.1f}, vy={vy:.1f}, vz={vz:.1f} m/s")
print()

# Set initial state
rocket.state.position = np.array([0.0, 0.0, 10_000.0])
rocket.state.velocity = np.array([vx, vy, vz])
rocket.state.orientation = np.array([0.0, angle_rad, 0.0])  # 70° pitch
rocket.state.angular_velocity = np.zeros(3)

# No thrust input
rocket.set_control_inputs(0.0, 0.0, 0.0)

# Simulate for 30 seconds
simulation_data = {
    'time': [],
    'position': [],
    'velocity': [],
    'altitude': [],
    'velocity_magnitude': [],
    'pitch': [],
}

dt = 0.01
max_time = 30.0
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
    
    # Update rocket
    rocket.update(dt)
    t += dt
    
    # Print progress
    if int(t) % 5 == 0 and t - dt < int(t):
        alt = rocket.state.position[2]
        vel = np.linalg.norm(rocket.state.velocity)
        pitch = np.degrees(rocket.state.orientation[1])
        vy_component = rocket.state.velocity[1]
        print(f"  Time: {t:6.2f}s | Alt: {alt:8.1f}m | Vel: {vel:7.1f}m/s | Pitch: {pitch:7.1f}° | Vy: {vy_component:7.1f}m/s")

print()
print(f"Simulation ended at t={t:.2f}s, altitude={rocket.state.position[2]:.1f}m")
print()

# Analysis
print("=" * 70)
print("ANALYSIS")
print("=" * 70)
print()

# Convert to numpy arrays for analysis
times = np.array(simulation_data['time'])
positions = np.array(simulation_data['position'])
velocities = np.array(simulation_data['velocity'])
altitudes = np.array(simulation_data['altitude'])
pitches = np.array(simulation_data['pitch'])
vel_mags = np.array(simulation_data['velocity_magnitude'])

# Find key points
max_alt_idx = np.argmax(altitudes)
max_alt = altitudes[max_alt_idx]
time_at_max_alt = times[max_alt_idx]
pitch_at_max_alt = pitches[max_alt_idx]

# Displacement
start_pos = positions[0]
end_pos = positions[-1]
displacement = end_pos - start_pos
horizontal_displacement = np.sqrt(displacement[0]**2 + displacement[1]**2)

print(f"Initial pitch angle: {pitches[0]:.1f}°")
print(f"Final pitch angle: {pitches[-1]:.1f}°")
print(f"Pitch change: {pitches[-1] - pitches[0]:.1f}°")
print()
print(f"Maximum altitude: {max_alt:.1f}m at t={time_at_max_alt:.1f}s (pitch: {pitch_at_max_alt:.1f}°)")
print()
print(f"Horizontal displacement: {horizontal_displacement:.1f}m")
print(f"  - X direction: {displacement[0]:.1f}m")
print(f"  - Y direction: {displacement[1]:.1f}m (should be positive, gliding right)")
print()

# Glide ratio analysis
vertical_drop = start_pos[2] - end_pos[2]
if vertical_drop > 0:
    glide_ratio = horizontal_displacement / vertical_drop
    print(f"Vertical drop: {vertical_drop:.1f}m")
    print(f"Glide ratio: {glide_ratio:.3f} (horizontal/vertical)")
    print()

# Final velocity direction
final_vel = velocities[-1]
final_vel_mag = vel_mags[-1]
if final_vel_mag > 0:
    print(f"Final velocity: {final_vel_mag:.1f}m/s")
    print(f"  - vx: {final_vel[0]:.1f}m/s")
    print(f"  - vy: {final_vel[1]:.1f}m/s (gliding direction)")
    print(f"  - vz: {final_vel[2]:.1f}m/s")
print()

# Plot results
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Aileron Test: Rocket Gliding Behavior', fontsize=16, fontweight='bold')

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
ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylabel('Pitch Angle (degrees)', fontweight='bold')
ax.set_title('Pitch Angle vs Time (Should trend toward 0° as gliding occurs)')
ax.grid(True, alpha=0.3)

# Trajectory in XY plane
ax = axes[1, 0]
ax.plot(positions[:, 0], positions[:, 1], 'g-', linewidth=2, label='Trajectory')
ax.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='Start')
ax.plot(positions[-1, 0], positions[-1, 1], 'rx', markersize=12, markeredgewidth=2, label='End')
ax.set_xlabel('X position (m)', fontweight='bold')
ax.set_ylabel('Y position (m)', fontweight='bold')
ax.set_title('Trajectory in XY plane (should curve toward +Y)')
ax.legend()
ax.grid(True, alpha=0.3)
ax.axis('equal')

# Altitude vs Y position (side view of gliding)
ax = axes[1, 1]
ax.plot(positions[:, 1], altitudes, 'b-', linewidth=2)
ax.plot(positions[0, 1], altitudes[0], 'go', markersize=10, label='Start')
ax.plot(positions[-1, 1], altitudes[-1], 'rx', markersize=12, markeredgewidth=2, label='End')
ax.set_xlabel('Y position (m)', fontweight='bold')
ax.set_ylabel('Altitude (m)', fontweight='bold')
ax.set_title('Altitude vs Y position (gliding path)')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('aileron_test.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'aileron_test.png'")
plt.show()
