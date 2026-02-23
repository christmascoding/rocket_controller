#!/usr/bin/env python3
"""
3D Visualization of rocket trajectory from simulation.
Creates an interactive 3D plot showing the flight path.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.config import CONFIG
from src.simulation.simulator import RocketSimulator

def plot_3d_trajectory():
    """Run simulation and create 3D trajectory visualization"""
    
    print("Running simulation...")
    sim = RocketSimulator(CONFIG)
    history = sim.run()
    print("Simulation completed!")
    
    # Extract history
    pos = np.array(history["position"])
    pos_ref = np.array(history["pos_ref"])
    attitude = np.array(history["attitude"])
    time = np.array(history["time"])
    gimbal = np.array(history.get("gimbal_angles", []))
    
    # Create 3D figure
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot reference trajectory (green dashed)
    ax.plot(pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2], 
            'g--', linewidth=2.5, alpha=0.7, label='Target Path')
    
    # Plot actual trajectory (orange/blue gradient based on time)
    # Color by altitude for better visualization
    scatter = ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], 
                        c=time, cmap='viridis', s=10, alpha=0.6)
    
    # Plot actual path as line
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 
            'orange', linewidth=1.5, alpha=0.5, label='Actual Path')
    
    # Mark start and end
    ax.scatter(*pos[0], color='green', s=200, marker='o', label='Launch', zorder=5)
    ax.scatter(*pos[-1], color='red', s=200, marker='x', label='Landing', zorder=5)
    
    # Mark some intermediate points with attitude vectors
    sample_indices = np.linspace(0, len(pos)-1, 10, dtype=int)
    for idx in sample_indices:
        p = pos[idx]
        if len(attitude) > idx:
            # Draw a small arrow showing pitch/yaw
            pitch = attitude[idx, 1] * np.pi / 180  # Convert to radians
            yaw = attitude[idx, 2] * np.pi / 180
            
            # Direction vector based on pitch and yaw
            dx = 100 * np.sin(yaw) * np.cos(pitch)
            dy = 100 * np.cos(yaw) * np.cos(pitch)
            dz = 100 * np.sin(pitch)
            
            ax.quiver(p[0], p[1], p[2], dx, dy, dz, 
                     color='blue', alpha=0.4, arrow_length_ratio=0.3, length=1)
    
    # Labels and formatting
    ax.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Z Altitude (m)', fontsize=11, fontweight='bold')
    ax.set_title('Rocket 3D Trajectory - Iteration 12 (Fixed)', fontsize=13, fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label('Time (seconds)', fontsize=10)
    
    # Legend
    ax.legend(loc='upper left', fontsize=10)
    
    # Set viewing angle
    ax.view_init(elev=25, azim=45)
    
    # Grid
    ax.grid(True, alpha=0.3)
    
    # Equal aspect ratio to avoid distortion
    max_range = max(
        np.max(pos[:, 0]) - np.min(pos[:, 0]),
        np.max(pos[:, 1]) - np.min(pos[:, 1]),
        np.max(pos[:, 2]) - np.min(pos[:, 2])
    ) / 2.0
    
    mid_x = (np.max(pos[:, 0]) + np.min(pos[:, 0])) * 0.5
    mid_y = (np.max(pos[:, 1]) + np.min(pos[:, 1])) * 0.5
    mid_z = (np.max(pos[:, 2]) + np.min(pos[:, 2])) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Save
    plt.savefig('3d_trajectory.png', dpi=150, bbox_inches='tight')
    print("\nSaved: 3d_trajectory.png")
    
    # Print stats
    print("\n" + "="*60)
    print("3D TRAJECTORY STATISTICS")
    print("="*60)
    print(f"Total distance traveled:   {np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)):.1f} m")
    print(f"Horizontal range (X):      {np.max(pos[:, 0]) - np.min(pos[:, 0]):.1f} m")
    print(f"Horizontal range (Y):      {np.max(pos[:, 1]) - np.min(pos[:, 1]):.1f} m")
    print(f"Vertical range (Z):        {np.max(pos[:, 2]) - np.min(pos[:, 2]):.1f} m")
    print(f"Trajectory duration:       {time[-1]:.1f} seconds")
    print("="*60 + "\n")
    
    plt.show()

if __name__ == '__main__':
    plot_3d_trajectory()
