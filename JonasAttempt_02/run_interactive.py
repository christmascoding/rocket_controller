#!/usr/bin/env python3
"""
Rocket Control Simulation - Interactive Playback
Computes the entire simulation, then displays with interactive controls
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.models.trajectory import VerticalLaunchPath
from src.simulation import RocketSimulation
from src.visualization.interactive_plotter import InteractiveRocket3DVisualizer

print("=" * 70)
print("ROCKET CONTROL SIMULATION - INTERACTIVE PLAYBACK")
print("=" * 70)
print()

# Create trajectory
trajectory = VerticalLaunchPath(target_altitude=60_000.0)

print(f"Trajectory: Vertical Launch to {trajectory.target_altitude/1000:.0f}km")
print(f"  - Required velocity: {trajectory.required_velocity:.1f}m/s")
print(f"  - Time to apogee: {trajectory.time_to_apogee:.1f}s")
print()

print("Computing simulation...")
print("(This will take a moment as it computes the entire flight)")
print()

# Create simulation WITHOUT visualization
sim = RocketSimulation(trajectory, visualize=False)

# Run entire simulation
telemetry = sim.run()

print()
print("Simulation complete! Loading interactive visualization...")
print()

# Create interactive visualizer with pre-computed data
visualizer = InteractiveRocket3DVisualizer(telemetry)

print("Controls:")
print("  - Slider: Jump to any point in the flight")
print("  - Play: Start/Stop playback")
print("  - Speed buttons (1x, 2x, 4x, 8x, 16x): Control playback speed")
print()

# Show interactive visualization
visualizer.show()
