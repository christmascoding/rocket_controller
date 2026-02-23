#!/usr/bin/env python3
"""
Test cascaded trajectory tracking controller with clean visualization.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from src.models.rocket import RocketState, Rocket
from src.models.trajectory import CurvedAscentPath
from src.controllers.cascaded_controller import CascadedRocketController
from src.visualization.static_trajectory import StaticTrajectoryGenerator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print("=" * 80)
print("CASCADED CONTROLLER TEST - PROPER TRAJECTORY TRACKING")
print("=" * 80)
print()

# Create trajectory reference
trajectory_ref = CurvedAscentPath(peak_altitude=60_000.0, horizontal_distance=40_000.0)
static_parabola = StaticTrajectoryGenerator(peak_altitude=60_000.0, horizontal_distance=40_000.0)

print(f"Reference trajectory:")
print(f"  Peak altitude: 60,000m")
print(f"  Horizontal distance: 40,000m")
print(f"  Time to peak: {trajectory_ref.t_peak:.1f}s")
print()

# Create rocket and controller
rocket = Rocket()
controller = CascadedRocketController()

# Initial state - match trajectory start
initial_state = trajectory_ref.get_desired_state(0.0)
rocket.state = RocketState(
    position=initial_state['position'].copy(),
    velocity=initial_state['velocity'].copy(),
    orientation=initial_state['orientation'].copy(),
    angular_velocity=np.zeros(3)
)

# Simulation data
telemetry = {
    'time': [],
    'position': [],
    'velocity': [],
    'orientation': [],
    'gimbal_pitch': [],
    'gimbal_yaw': [],
    'thrust': [],
}

dt = 0.01
time = 0.0
max_time = 140.0

print("Simulating with cascaded controller...")
print(f"{'Time(s)':>8} | {'Altitude(km)':>12} | {'Target(km)':>12} | {'Error(km)':>10} | "
    f"{'Vz(m/s)':>9} | {'Throttle':>8} | {'Gimbal P(°)':>10} | {'Gimbal Y(°)':>10}")
print("-" * 80)

step = 0
gimbal_saturation_count = 0
throttle_saturation_count = 0
max_abs_gimbal = 0.0
max_abs_vz = 0.0
while time < max_time:
    # Get desired state from reference trajectory
    desired = trajectory_ref.get_desired_state(time)
    
    # Compute control inputs using cascaded controller
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(
        rocket.state,
        desired,
        dt
    )
    
    # Print every 10 seconds
    if step % (int(10.0 / dt)) == 0:
        error_z = (rocket.state.position[2] - desired['position'][2]) / 1000.0
        print(f"{time:8.1f} | {rocket.state.position[2]/1000:12.2f} | {desired['position'][2]/1000:12.2f} | "
              f"{error_z:10.3f} | {rocket.state.velocity[2]:9.1f} | {thrust:8.3f} | {np.degrees(gimbal_pitch):10.2f} | {np.degrees(gimbal_yaw):10.2f}")

    # Saturation diagnostics
    gimbal_limit_deg = np.degrees(controller.stabilization.gimbal_max)
    if abs(np.degrees(gimbal_pitch)) >= gimbal_limit_deg - 0.1 or abs(np.degrees(gimbal_yaw)) >= gimbal_limit_deg - 0.1:
        gimbal_saturation_count += 1
    if thrust >= 0.999 or thrust <= 0.001:
        throttle_saturation_count += 1
    max_abs_gimbal = max(max_abs_gimbal, abs(np.degrees(gimbal_pitch)), abs(np.degrees(gimbal_yaw)))
    max_abs_vz = max(max_abs_vz, abs(rocket.state.velocity[2]))
    
    # Record telemetry
    telemetry['time'].append(time)
    telemetry['position'].append(rocket.state.position.copy())
    telemetry['velocity'].append(rocket.state.velocity.copy())
    telemetry['orientation'].append(rocket.state.orientation.copy())
    telemetry['gimbal_pitch'].append(gimbal_pitch)
    telemetry['gimbal_yaw'].append(gimbal_yaw)
    telemetry['thrust'].append(thrust)
    
    # Apply control inputs
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(dt)
    
    time += dt
    step += 1

print()
print("=" * 80)
print("SIMULATION COMPLETE")
print("=" * 80)
print(f"Final altitude: {rocket.state.position[2]/1000:.2f}km (target: 60.00km)")
print(f"Final X position: {rocket.state.position[0]/1000:.2f}km (target: 40.00km)")
print(f"Tracking error at peak time: {(rocket.state.position[2] - 60_000.0)/1000:.2f}km")
print(f"Gimbal saturation count: {gimbal_saturation_count} / {step} steps")
print(f"Throttle saturation count: {throttle_saturation_count} / {step} steps")
print(f"Max |gimbal|: {max_abs_gimbal:.2f}°")
print(f"Max |Vz|: {max_abs_vz:.1f} m/s")
print()

# Create 3D visualization
print("Creating 3D visualization...")
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# Draw static parabola
parabola_traj = static_parabola.trajectory_array
ax.plot(parabola_traj[:, 0], parabola_traj[:, 1], parabola_traj[:, 2],
        'b-', linewidth=3.0, alpha=0.9, label='Target Parabola', zorder=10)

# Draw actual trajectory with heatmap coloring
actual_traj = np.array(telemetry['position'])
times = np.array(telemetry['time'])
scatter = ax.scatter(actual_traj[:, 0], actual_traj[:, 1], actual_traj[:, 2],
                     c=times, cmap='hot', s=3, alpha=0.7, label='Actual Path', zorder=5)

# Colorbar for time
cbar = plt.colorbar(scatter, ax=ax, label='Time (s)', pad=0.1, shrink=0.8)

# Mark start and end
ax.scatter(*actual_traj[0], color='green', s=200, marker='o', label='Launch', zorder=15)
ax.scatter(*actual_traj[-1], color='red', s=200, marker='x', label='Final', zorder=15)

# Setup axes
ax.set_xlabel('X Position (m)', fontweight='bold')
ax.set_ylabel('Y Position (m)', fontweight='bold')
ax.set_zlabel('Z Altitude (m)', fontweight='bold')
ax.set_title('Cascaded Controller - Trajectory Tracking', fontsize=13, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.legend(loc='upper left', fontsize=10)
ax.view_init(elev=20, azim=45)

plt.tight_layout()
plt.savefig('cascaded_trajectory.png', dpi=150, bbox_inches='tight')
print("Saved: cascaded_trajectory.png")
plt.show()
