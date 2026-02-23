#!/usr/bin/env python3
"""
Rocket Control Simulation - Main Launch Script

This script demonstrates the rocket control system with a vertical launch trajectory.
The rocket will accelerate upward to reach a target altitude of 40km.

Features:
- 6-DOF rigid body dynamics
- Gimbaled engine control
- Automatic attitude controller
- Real-time 3D visualization
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.models.trajectory import VerticalLaunchPath, SpiralPath, ParabolicPath
from src.simulation import RocketSimulation
from src.config import RocketSpecs

def run_vertical_launch():
    """Run simulation with vertical launch to 40km."""
    print("=" * 70)
    print("ROCKET CONTROL SIMULATION - VERTICAL LAUNCH TO 40KM")
    print("=" * 70)
    print()
    print("Rocket Specifications:")
    print("  - Diameter: 70cm")
    print("  - Length: 5.825m")
    print("  - Mass: 10 tonnes")
    print("  - Max Thrust: 300kN")
    print("  - Gimbal Range: ±15°")
    print("  - CG to Engine: 2.8m")
    print()
    
    # Create vertical launch trajectory
    trajectory = VerticalLaunchPath(target_altitude=40_000.0)
    
    print(f"Trajectory: Vertical Launch to {trajectory.target_altitude/1000:.0f}km")
    print(f"  - Required velocity: {trajectory.required_velocity:.1f}m/s")
    print(f"  - Time to apogee: {trajectory.time_to_apogee:.1f}s")
    print()
    
    # Create and run simulation
    sim = RocketSimulation(trajectory, visualize=True)
    telemetry = sim.run()
    
    return telemetry


def run_spiral_launch():
    """Run simulation with spiral trajectory."""
    print("=" * 70)
    print("ROCKET CONTROL SIMULATION - SPIRAL LAUNCH TO 40KM")
    print("=" * 70)
    print()
    
    # Create spiral trajectory
    trajectory = SpiralPath(target_altitude=40_000.0, num_spirals=3)
    
    print(f"Trajectory: Spiral Launch to {trajectory.target_altitude/1000:.0f}km")
    print(f"  - Number of spirals: {trajectory.num_spirals}")
    print(f"  - Required velocity: {trajectory.required_velocity:.1f}m/s")
    print()
    
    # Create and run simulation
    sim = RocketSimulation(trajectory, visualize=True)
    telemetry = sim.run()
    
    return telemetry


def run_parabolic_launch():
    """Run simulation with parabolic trajectory."""
    print("=" * 70)
    print("ROCKET CONTROL SIMULATION - PARABOLIC LAUNCH")
    print("=" * 70)
    print()
    
    # Create parabolic trajectory
    trajectory = ParabolicPath(target_altitude=10_000.0, horizontal_range=5_000.0)
    
    print(f"Trajectory: Parabolic Launch")
    print(f"  - Target altitude: {trajectory.target_altitude:.1f}m")
    print(f"  - Horizontal range: {trajectory.horizontal_range:.1f}m")
    print()
    
    # Create and run simulation
    sim = RocketSimulation(trajectory, visualize=True)
    telemetry = sim.run()
    
    return telemetry


if __name__ == "__main__":
    print("\n")
    print("=" * 70)
    print("  ROCKET CONTROL SYSTEM - GIMBALED ENGINE SIMULATION".center(70))
    print("=" * 70)
    print()
    
    # You can change which trajectory to run by commenting/uncommenting
    
    # Run vertical launch (default)
    telemetry = run_vertical_launch()
    
    # Uncomment below to run spiral launch instead
    # telemetry = run_spiral_launch()
    
    # Uncomment below to run parabolic launch instead
    # telemetry = run_parabolic_launch()
    
    print("\nSimulation complete! Check the visualization window for the 3D trajectory.")
