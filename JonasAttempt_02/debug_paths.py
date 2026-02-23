#!/usr/bin/env python3
"""
Debug test - check if target path is being populated
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.models.trajectory import VerticalLaunchPath
from src.simulation import RocketSimulation

print("Running short simulation to check paths...")

# Create trajectory
trajectory = VerticalLaunchPath(target_altitude=40_000.0)

# Create simulation WITHOUT visualization to test logic
sim = RocketSimulation(trajectory, visualize=True)

# Run just 100 steps
for i in range(100):
    if not sim.step():
        break

print(f"\nAfter 100 steps:")
print(f"Actual path points: {len(sim.visualizer.trajectory_history)}")
print(f"Desired path points: {len(sim.visualizer.desired_trajectory_history)}")
print(f"Current time: {sim.current_time:.2f}s")

if len(sim.visualizer.trajectory_history) > 0:
    print(f"First actual position: {sim.visualizer.trajectory_history[0]}")

if len(sim.visualizer.desired_trajectory_history) > 0:
    print(f"First desired position: {sim.visualizer.desired_trajectory_history[0]}")
else:
    print("WARNING: No desired trajectory points recorded!")
