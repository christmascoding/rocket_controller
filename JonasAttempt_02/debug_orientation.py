#!/usr/bin/env python3
"""
Debug rocket orientation vs desired trajectory.
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

print("="*120)
print("ORIENTATION DEBUG - ROCKET VS DESIRED TRAJECTORY")
print("="*120)
print()

# Get initial states
initial_state = trajectory.get_desired_state(0.0)
rocket = Rocket()
rocket.state = RocketState(
    position=initial_state['position'].copy(),
    velocity=initial_state['velocity'].copy(),
    orientation=initial_state['orientation'].copy(),
    angular_velocity=np.array([0.0, 0.0, 0.0])
)

print(f"INITIAL CONDITIONS (t=0)")
print(f"{'='*120}")
print(f"Rocket position:     {rocket.state.position}")
print(f"Rocket velocity:     {rocket.state.velocity}")
print(f"Rocket orientation (roll, pitch, yaw): {np.degrees(rocket.state.orientation)}°")
print()
print(f"Desired position:    {initial_state['position']}")
print(f"Desired velocity:    {initial_state['velocity']}")
print(f"Desired orientation (roll, pitch, yaw): {np.degrees(initial_state['orientation'])}°")
print()

# Calculate what pitch angle SHOULD be based on velocity vector
desired_vel = initial_state['velocity']
desired_pitch_from_vel = np.arctan2(desired_vel[2], np.sqrt(desired_vel[0]**2 + desired_vel[1]**2))
print(f"Desired pitch from velocity vector: {np.degrees(desired_pitch_from_vel):.2f}°")
print(f"Expected: arctan(VZ/sqrt(VX^2+VY^2)) = arctan({desired_vel[2]:.2f}/{np.sqrt(desired_vel[0]**2 + desired_vel[1]**2):.2f})")
print()

controller = RocketController()

# Simulate for first 150 seconds, print every 10 seconds
dt = 0.01
time = 0.0
max_time = 150.0

print(f"{'Time(s)':>8} | {'Rocket Pitch(°)':>15} | {'Desired Pitch(°)':>15} | {'Error(°)':>10} | {'Gimbal Pitch(°)':>15} | {'Rocket Pos Z(km)':>15} | {'Desired Z(km)':>15}")
print("-"*120)

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
        rocket_pitch = np.degrees(rocket.state.orientation[1])
        desired_pitch = np.degrees(desired['orientation'][1])
        pitch_error = rocket_pitch - desired_pitch
        
        # Normalize error to [-180, 180]
        while pitch_error > 180:
            pitch_error -= 360
        while pitch_error < -180:
            pitch_error += 360
        
        print(f"{time:8.1f} | {rocket_pitch:15.2f} | {desired_pitch:15.2f} | {pitch_error:10.2f} | "
              f"{np.degrees(gimbal_pitch):15.2f} | {rocket.state.position[2]/1000:15.2f} | {desired['position'][2]/1000:15.2f}")
    
    # Apply control
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(dt)
    
    time += dt
    step += 1

print()
print("="*120)
print("ANALYSIS")
print("="*120)
print()
print("Key observations:")
print("1. If Rocket Pitch is negative and Desired is positive, rocket is pointing DOWN while target points UP")
print("2. If gimbal is always ±15°, it's maxed out - controller gains too high")
print("3. If Rocket Z doesn't increase towards Desired Z, the controller isn't working")
print()
