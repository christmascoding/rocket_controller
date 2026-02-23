#!/usr/bin/env python3
"""
Full debug of trajectory showing EVERY detail.
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

print("="*100)
print("TRAJECTORY DEFINITION")
print("="*100)
print(f"Peak altitude: {trajectory.peak_altitude}m")
print(f"Horizontal distance: {trajectory.horizontal_distance}m")
print(f"v0_z (initial vertical velocity): {trajectory.v0_z:.2f}m/s")
print(f"v0_x (initial horizontal velocity): {trajectory.v0_x:.2f}m/s")
print(f"t_peak (time to reach peak): {trajectory.t_peak:.2f}s")
print(f"Phase 1 duration: {trajectory.phase1_duration:.2f}s")
print(f"Phase 2 duration: {trajectory.phase2_duration:.2f}s")
print(f"Total time: {trajectory.total_time}s")
print()

# Print first few desired states
print("="*100)
print("DESIRED TRAJECTORY SAMPLES (FIRST 20 SECONDS)")
print("="*100)
print(f"{'Time(s)':>8} | {'X(m)':>10} | {'Z(m)':>10} | {'VX(m/s)':>10} | {'VZ(m/s)':>10} | {'Pitch(rad)':>12} | {'Yaw(rad)':>12}")
print("-"*100)

for t in np.linspace(0, 20, 21):
    state = trajectory.get_desired_state(t)
    pos = state['position']
    vel = state['velocity']
    ori = state['orientation']
    print(f"{t:8.2f} | {pos[0]:10.1f} | {pos[2]:10.1f} | {vel[0]:10.2f} | {vel[2]:10.2f} | {ori[1]:12.4f} | {ori[2]:12.4f}")

print()
print("="*100)
print("TRAJECTORY PROGRESSION - KEY MILESTONES")
print("="*100)
print(f"{'Time(s)':>8} | {'X(m)':>10} | {'Z(m)':>10} | {'VX(m/s)':>10} | {'VZ(m/s)':>10}")
print("-"*100)

for t in [0, 10, 20, 30, 50, 60, 80, 100, 110.6, 120, 140]:
    if t <= trajectory.total_time:
        state = trajectory.get_desired_state(t)
        pos = state['position']
        vel = state['velocity']
        print(f"{t:8.2f} | {pos[0]:10.1f} | {pos[2]:10.1f} | {vel[0]:10.2f} | {vel[2]:10.2f}")

print()
print("="*100)
print("NOW RUNNING SIMULATION WITH DETAILED OUTPUT")
print("="*100)
print()

# Initialize rocket to match trajectory start
initial_state = trajectory.get_desired_state(0.0)
rocket = Rocket()
rocket.state = RocketState(
    position=initial_state['position'].copy(),
    velocity=initial_state['velocity'].copy(),
    orientation=initial_state['orientation'].copy(),
    angular_velocity=np.array([0.0, 0.0, 0.0])
)

controller = RocketController()

dt = 0.01
time = 0.0
max_time = 140.0

step = 0
print(f"{'Time(s)':>8} | {'ACTUAL':^40} | {'DESIRED':^40} | {'ERROR':^20} | {'Gimbal(deg)':^20}")
print(f"{'':>8} | {'X(m)':>12} {'Z(m)':>12} {'VZ(m/s)':>12} | {'X(m)':>12} {'Z(m)':>12} {'VZ(m/s)':>12} | {'dX(km)':>10} {'dZ(km)':>10} | {'Pitch':>10} {'Yaw':>10}")
print("-"*120)

while time < max_time:
    # Get desired state
    desired = trajectory.get_desired_state(time)
    
    # Compute control
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(
        rocket.state,
        desired,
        dt
    )
    
    # Print every 100 steps (approximately every 1 second)
    if step % 100 == 0:
        actual_pos = rocket.state.position
        actual_vel = rocket.state.velocity
        desired_pos = desired['position']
        desired_vel = desired['velocity']
        
        error_x = (actual_pos[0] - desired_pos[0]) / 1000.0  # in km
        error_z = (actual_pos[2] - desired_pos[2]) / 1000.0  # in km
        
        print(f"{time:8.2f} | {actual_pos[0]:12.1f} {actual_pos[2]:12.1f} {actual_vel[2]:12.2f} | "
              f"{desired_pos[0]:12.1f} {desired_pos[2]:12.1f} {desired_vel[2]:12.2f} | "
              f"{error_x:10.3f} {error_z:10.3f} | {np.degrees(gimbal_pitch):10.2f} {np.degrees(gimbal_yaw):10.2f}")
    
    # Apply control
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(dt)
    
    time += dt
    step += 1

print()
print("="*100)
print("SIMULATION COMPLETE")
print("="*100)
print(f"Final position: X={rocket.state.position[0]:.1f}m, Z={rocket.state.position[2]:.1f}m")
print(f"Final velocity: VX={rocket.state.velocity[0]:.2f}m/s, VZ={rocket.state.velocity[2]:.2f}m/s")
