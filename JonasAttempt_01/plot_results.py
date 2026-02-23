#!/usr/bin/env python3
"""
Simple visualization of rocket trajectory results.
Loads history from simulation and creates plot figures.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import pickle
import os

def plot_trajectory():
    """Plot the complete rocket trajectory from simulation results"""
    
    # Check if history file exists
    history_file = 'simulation_history.pkl'
    if not os.path.exists(history_file):
        print(f"No {history_file} found. Run simulation first with debug output.")
        return
    
    # Load history
    with open(history_file, 'rb') as f:
        history = pickle.load(f)
    
    # Extract data
    times = np.array(history['time'])
    positions = np.array(history['position'])  # [x, y] pairs
    velocities = np.array(history['velocity'])  # [vx, vy] pairs
    attitudes = np.array(history['attitude'])  # [roll, pitch, yaw]
    gimbal = np.array(history['gimbal'])  # [pitch, yaw]
    thrust = np.array(history['thrust'])
    gimbal_cmd = np.array(history['gimbal_cmd'])
    
    # Create comprehensive figure
    fig = plt.figure(figsize=(16, 12))
    
    # 1. Trajectory plot
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Trajectory')
    ax1.scatter(positions[0, 0], positions[0, 1], c='green', s=100, marker='o', label='Launch', zorder=5)
    ax1.scatter(positions[-1, 0], positions[-1, 1], c='red', s=100, marker='x', label='Final', zorder=5)
    ax1.set_xlabel('Horizontal Position (m)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Rocket Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_aspect('equal')
    
    # 2. Altitude over time
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(times, positions[:, 1], 'b-', linewidth=2)
    ax2.axhline(y=500, color='r', linestyle='--', alpha=0.5, label='Apogee target')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 3. Velocity components
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(times, velocities[:, 0], 'r-', linewidth=2, label='Horizontal Velocity')
    ax3.plot(times, velocities[:, 1], 'b-', linewidth=2, label='Vertical Velocity')
    ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Velocity Components')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 4. Pitch attitude
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(times, attitudes[:, 1], 'b-', linewidth=2, label='Pitch Angle')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Level')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Pitch Angle (degrees)')
    ax4.set_title('Pitch Attitude')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    # 5. Gimbal deflection
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(times, gimbal[:, 0], 'b-', linewidth=2, label='Gimbal Pitch (actual)')
    ax5.plot(times, gimbal_cmd[:, 0], 'r--', linewidth=1.5, alpha=0.7, label='Gimbal Pitch (commanded)')
    ax5.axhline(y=5, color='k', linestyle='--', alpha=0.3, label='Max Authority (±5°)')
    ax5.axhline(y=-5, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Gimbal Angle (degrees)')
    ax5.set_title('Gimbal Deflection')
    ax5.grid(True, alpha=0.3)
    ax5.legend()
    
    # 6. Thrust
    ax6 = plt.subplot(2, 3, 6)
    ax6.plot(times, thrust, 'g-', linewidth=2)
    ax6.axhline(y=490, color='k', linestyle='--', alpha=0.5, label='Gravity Comp (490N)')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Thrust (N)')
    ax6.set_title('Thrust Output')
    ax6.grid(True, alpha=0.3)
    ax6.legend()
    
    plt.tight_layout()
    plt.savefig('rocket_trajectory.png', dpi=150, bbox_inches='tight')
    print("Saved: rocket_trajectory.png")
    
    # Print summary statistics
    print("\n" + "="*60)
    print("ROCKET SIMULATION SUMMARY")
    print("="*60)
    print(f"Duration:              {times[-1]:.2f} seconds")
    print(f"\nTrajectory:")
    print(f"  Launch altitude:     {positions[0, 1]:.1f} m")
    print(f"  Maximum altitude:    {np.max(positions[:, 1]):.1f} m")
    print(f"  Final altitude:      {positions[-1, 1]:.1f} m")
    print(f"  Horizontal range:    {positions[-1, 0]:.1f} m")
    print(f"\nVelocity:")
    print(f"  Launch speed:        {np.linalg.norm(velocities[0]):.2f} m/s")
    print(f"  Final speed:         {np.linalg.norm(velocities[-1]):.2f} m/s")
    print(f"  Max speed:           {np.max(np.linalg.norm(velocities, axis=1)):.2f} m/s")
    print(f"\nAttitude:")
    print(f"  Max pitch:           {np.max(np.abs(attitudes[:, 1])):.2f}°")
    print(f"  Final pitch:         {attitudes[-1, 1]:.2f}°")
    print(f"\nActuation:")
    print(f"  Max gimbal cmd:      {np.max(np.abs(gimbal_cmd[:, 0])):.2f}°")
    print(f"  Max gimbal actual:   {np.max(np.abs(gimbal[:, 0])):.2f}°")
    print(f"  Avg thrust:          {np.mean(thrust):.1f} N")
    print("="*60)

if __name__ == '__main__':
    plot_trajectory()
    plt.show()
