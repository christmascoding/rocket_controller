"""
Clean trajectory rendering system.
Generates and renders the target parabola as a static, unchanging reference line.
"""

import numpy as np


class StaticTrajectoryGenerator:
    """Generates a static reference trajectory parabola."""
    
    def __init__(self, peak_altitude=60_000.0, horizontal_distance=40_000.0, num_points=500):
        """
        Generate a ballistic parabola trajectory.
        
        Args:
            peak_altitude: Peak altitude in meters (60km)
            horizontal_distance: Horizontal distance at peak in meters (40km)
            num_points: Number of discretization points
        """
        self.peak_altitude = peak_altitude
        self.horizontal_distance = horizontal_distance
        self.num_points = num_points
        
        # Compute parabola parameters
        self.v0_z = np.sqrt(2 * 9.81 * peak_altitude)
        self.t_peak = self.v0_z / 9.81
        self.v0_x = horizontal_distance / self.t_peak
        
        # Generate parabola
        self.trajectory_array = self._generate_parabola()
    
    def _generate_parabola(self):
        """
        Generate parabola trajectory array.
        
        Returns:
            Array of shape (num_points, 3) with columns [X, Y, Z]
        """
        times = np.linspace(0, self.t_peak, self.num_points)
        
        trajectory = np.zeros((self.num_points, 3))
        trajectory[:, 0] = self.v0_x * times        # X: horizontal
        trajectory[:, 1] = np.zeros_like(times)      # Y: zero (2D trajectory)
        trajectory[:, 2] = self.v0_z * times - 0.5 * 9.81 * times**2  # Z: ballistic
        
        return trajectory
    
    def get_target_at_time(self, current_time):
        """
        Get the target position at a specific time on the parabola.
        
        Args:
            current_time: Current simulation time (seconds)
        
        Returns:
            Target position [X, Y, Z] or None if time exceeds trajectory
        """
        if current_time > self.t_peak:
            # Maintain peak altitude after reaching it
            return np.array([
                self.v0_x * self.t_peak,
                0.0,
                self.peak_altitude
            ])
        
        if current_time < 0:
            return np.array([0.0, 0.0, 0.0])
        
        # Compute position on parabola
        x = self.v0_x * current_time
        z = self.v0_z * current_time - 0.5 * 9.81 * current_time**2
        
        return np.array([x, 0.0, z])


def setup_3d_plot_with_parabola(ax, trajectory_generator):
    """
    Set up a 3D plot with the static parabola trajectory.
    
    Args:
        ax: Matplotlib 3D axis
        trajectory_generator: StaticTrajectoryGenerator instance
    
    Returns:
        Plot line handle for the parabola
    """
    # Extract trajectory array
    traj = trajectory_generator.trajectory_array
    
    # Plot as solid blue line (the target to follow)
    parabola_line, = ax.plot(
        traj[:, 0],
        traj[:, 1],
        traj[:, 2],
        'b-',
        linewidth=3.0,
        alpha=0.9,
        label='Target Parabola (60km peak)',
        zorder=10
    )
    
    # Configure axes
    ax.set_xlabel('X Position (m)', fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontweight='bold')
    ax.set_zlabel('Z Altitude (m)', fontweight='bold')
    ax.set_title('Rocket Trajectory Tracking', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=10)
    
    return parabola_line
