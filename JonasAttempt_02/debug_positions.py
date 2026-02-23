#!/usr/bin/env python3
import sys, numpy as np
sys.path.insert(0, '.')
from src.models.rocket import Rocket
from src.models.trajectory import CurvedAscentPath
from src.controllers.controller import RocketController

traj = CurvedAscentPath()
rocket = Rocket()
controller = RocketController()

# Init rocket to match trajectory start
s = traj.get_desired_state(0)
rocket.state.position = s['position'].copy()
rocket.state.velocity = s['velocity'].copy()
rocket.state.orientation = s['orientation'].copy()

print(f'Initial state: pos={rocket.state.position}, vel={rocket.state.velocity}')
print()

# Simulate a few steps
for step in range(2000):  # 20 seconds
    desired = traj.get_desired_state(step * 0.01)
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(rocket.state, desired, 0.01)
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(0.01)

print(f'After 20 seconds of simulation:')
print(f'  Actual:  x={rocket.state.position[0]:.0f}m, y={rocket.state.position[1]:.0f}m, z={rocket.state.position[2]:.0f}m')
s = traj.get_desired_state(20.0)
print(f'  Desired: x={s["position"][0]:.0f}m, y={s["position"][1]:.0f}m, z={s["position"][2]:.0f}m')
print()

# Run to 40s
for step in range(2000, 4000):  
    desired = traj.get_desired_state(step * 0.01)
    thrust, gimbal_yaw, gimbal_pitch = controller.compute_control(rocket.state, desired, 0.01)
    rocket.set_control_inputs(thrust, gimbal_yaw, gimbal_pitch)
    rocket.update(0.01)

print(f'After 40 seconds of simulation:')
print(f'  Actual:  x={rocket.state.position[0]:.0f}m, y={rocket.state.position[1]:.0f}m, z={rocket.state.position[2]:.0f}m')
s = traj.get_desired_state(40.0)
print(f'  Desired: x={s["position"][0]:.0f}m, y={s["position"][1]:.0f}m, z={s["position"][2]:.0f}m')
