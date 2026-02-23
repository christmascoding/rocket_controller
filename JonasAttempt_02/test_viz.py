#!/usr/bin/env python3
"""
Quick test visualization of rocket flight
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use('TkAgg')  # Use interactive backend
import matplotlib.pyplot as plt

from src.models.trajectory import VerticalLaunchPath
from src.simulation import RocketSimulation

print("Starting rocket simulation with visualization...")
print("This will display a 3D plot of the rocket's flight.\n")

# Create trajectory
trajectory = VerticalLaunchPath(target_altitude=40_000.0)

# Create and run simulation with visualization
sim = RocketSimulation(trajectory, visualize=True)

print("Running simulation - check the visualization window...")
telemetry = sim.run()

print("\nSimulation complete!")
print(f"Final altitude reached: {max(sim.telemetry['altitude']):.1f}m")
print(f"Final velocity: {sim.telemetry['velocity_magnitude'][-1]:.1f}m/s")
