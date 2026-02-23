"""
3D Visualization System for Rocket Simulation
Creates dual rotating 3D plots:
1. Zoomed view of rocket with gimbal thrust visualization
2. Full trajectory view
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.config import RocketSpecs, VisualizationParams

class RocketVisualizer3D:
    """Handles 3D visualization of rocket and trajectory."""
    
    def __init__(self, specs=None, figsize=(16, 8)):
        """
        Initialize visualizer with dual 3D plots.
        
        Args:
            specs: RocketSpecs object
            figsize: Figure size (width, height)
        """
        self.specs = specs or RocketSpecs()
        
        # Create figure with two subplots side by side
        self.fig = plt.figure(figsize=figsize)
        self.fig.suptitle('Rocket Control Simulation - 3D View', fontsize=16, fontweight='bold')
        
        # Left plot: Zoomed rocket view
        self.ax_rocket = self.fig.add_subplot(121, projection='3d')
        
        # Right plot: Full trajectory view
        self.ax_trajectory = self.fig.add_subplot(122, projection='3d')
        
        # Initialize plot handles
        self.rocket_body_line = None
        self.rocket_nose_cone = None
        self.gimbal_thrust_line = None
        self.cg_point = None
        self.coordinate_axes = {'x': None, 'y': None, 'z': None}
        
        self.trajectory_line = None
        self.trajectory_current_pos = None
        self.trajectory_launch_point = None
        self.desired_trajectory_line = None
        
        # Trajectory history (actual and desired)
        self.trajectory_history = []
        self.desired_trajectory_history = []
        self.thrust_history = []
        self.gimbal_angle_history = []
        
        # Rotation angle for automatic rotation
        self.rotation_angle = 0.0
        
        self._setup_rocket_plot()
        self._setup_trajectory_plot()
    
    def _setup_rocket_plot(self):
        """Setup the zoomed rocket view plot."""
        ax = self.ax_rocket
        ax.set_xlabel('X (m)', fontweight='bold')
        ax.set_ylabel('Y (m)', fontweight='bold')
        ax.set_zlabel('Z (m)', fontweight='bold')
        ax.set_title('Rocket Detail View (CG at Origin)', fontsize=12, fontweight='bold')
        
        # Set limits for detailed view (±10m around rocket)
        limit = 15
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.set_zlim([-limit, limit])
        
        ax.grid(True, alpha=0.3)
        ax.set_box_aspect([1, 1, 1])
    
    def _setup_trajectory_plot(self):
        """Setup the full trajectory view plot."""
        ax = self.ax_trajectory
        ax.set_xlabel('X (m)', fontweight='bold')
        ax.set_ylabel('Y (m)', fontweight='bold')
        ax.set_zlabel('Z (m)', fontweight='bold')
        ax.set_title('Trajectory View', fontsize=12, fontweight='bold')
        
        # Will adjust limits dynamically
        ax.grid(True, alpha=0.3)
        ax.set_box_aspect([1, 1, 1])
    
    def _compute_rocket_geometry(self, rocket_state):
        """
        Compute rocket body geometry in inertial frame.
        
        Args:
            rocket_state: RocketState object
        
        Returns:
            Dictionary with rocket geometry points
        """
        # Create rotation matrix from orientation angles
        alpha1, alpha2, alpha3 = rocket_state.orientation
        c1, s1 = np.cos(alpha1), np.sin(alpha1)
        c2, s2 = np.cos(alpha2), np.sin(alpha2)
        c3, s3 = np.cos(alpha3), np.sin(alpha3)
        
        # ZYX rotation matrix
        R = np.array([
            [c3*c2, c3*s2*s1 - s3*c1, c3*s2*c1 + s3*s1],
            [s3*c2, s3*s2*s1 + c3*c1, s3*s2*c1 - c3*s1],
            [-s2,   c2*s1,            c2*c1]
        ])
        
        # Rocket body points (in body frame, z-axis points along rocket axis)
        # CG is at the origin of the body frame
        body_length = self.specs.length
        cg_offset = self.specs.engine_offset_from_cg
        
        # Nose cone tip (at distance body_length/2 from CG)
        nose_tip_body = np.array([0.0, 0.0, body_length / 2])
        
        # Engine gimbal point (at distance -cg_offset from CG)
        engine_point_body = np.array([0.0, 0.0, -cg_offset])
        
        # Rocket base (at distance -body_length/2 from CG)
        rocket_base_body = np.array([0.0, 0.0, -body_length / 2])
        
        # Transform to inertial frame (CG is at origin)
        nose_tip = R @ nose_tip_body
        engine_point = R @ engine_point_body
        rocket_base = R @ rocket_base_body
        
        # Rocket radius for visualization
        radius = self.specs.diameter / 2
        
        return {
            'nose_tip': nose_tip,
            'engine_point': engine_point,
            'rocket_base': rocket_base,
            'radius': radius,
            'rotation_matrix': R
        }
    
    def _draw_rocket_body(self, rocket_state, thrust, gimbal_pitch, gimbal_yaw):
        """
        Draw the rocket body and thrust vector in the rocket detail view.
        
        Args:
            rocket_state: RocketState object
            thrust: Normalized thrust (0-1)
            gimbal_pitch: Gimbal pitch angle (radians)
            gimbal_yaw: Gimbal yaw angle (radians)
        """
        ax = self.ax_rocket
        
        # Clear previous rocket elements safely
        if self.rocket_body_line is not None:
            try:
                self.rocket_body_line.remove()
            except:
                pass
        if self.rocket_nose_cone is not None:
            try:
                self.rocket_nose_cone.remove()
            except:
                pass
        if self.gimbal_thrust_line is not None:
            try:
                self.gimbal_thrust_line.remove()
            except:
                pass
        if self.cg_point is not None:
            try:
                self.cg_point.remove()
            except:
                pass
        for line in self.coordinate_axes.values():
            if line is not None:
                try:
                    line.remove()
                except:
                    pass
        
        # Get rocket geometry
        geom = self._compute_rocket_geometry(rocket_state)
        
        # Draw rocket body (main cylinder)
        nose = geom['nose_tip']
        base = geom['rocket_base']
        
        # Rocket body as a line
        self.rocket_body_line, = ax.plot(
            [base[0], nose[0]],
            [base[1], nose[1]],
            [base[2], nose[2]],
            'b-', linewidth=3, label='Rocket Body'
        )
        
        # Draw nose cone (small cone at tip)
        cone_base_size = geom['radius'] * 0.3
        cone_points = []
        for angle in np.linspace(0, 2*np.pi, 8):
            # Create a small circle around the nose tip
            offset_body = np.array([
                cone_base_size * np.cos(angle),
                cone_base_size * np.sin(angle),
                0.0
            ])
            offset_inertial = geom['rotation_matrix'] @ offset_body
            cone_points.append(nose + offset_inertial)
        
        cone_points = np.array(cone_points)
        for i in range(len(cone_points)):
            next_i = (i + 1) % len(cone_points)
            ax.plot(
                [cone_points[i, 0], nose[0], cone_points[next_i, 0]],
                [cone_points[i, 1], nose[1], cone_points[next_i, 1]],
                [cone_points[i, 2], nose[2], cone_points[next_i, 2]],
                'b-', linewidth=1, alpha=0.5
            )
        
        # Draw gimbal thrust vector
        engine_point = geom['engine_point']
        
        # Compute thrust vector direction (with gimbal deflection)
        # In body frame
        thrust_length = VisualizationParams.thrust_line_min_length + \
                       thrust * (VisualizationParams.thrust_line_max_length - VisualizationParams.thrust_line_min_length)
        
        # Thrust direction affected by gimbal angles
        thrust_dir_body = np.array([
            np.sin(gimbal_yaw),
            np.sin(gimbal_pitch),
            -np.cos(gimbal_pitch) * np.cos(gimbal_yaw)
        ])
        
        # Normalize and scale
        thrust_dir_body = thrust_dir_body / np.linalg.norm(thrust_dir_body) * thrust_length
        thrust_dir_inertial = geom['rotation_matrix'] @ thrust_dir_body
        
        thrust_endpoint = engine_point + thrust_dir_inertial
        
        # Color based on thrust level
        color = [
            VisualizationParams.thrust_min_color[i] * (1 - thrust) + 
            VisualizationParams.thrust_max_color[i] * thrust
            for i in range(3)
        ]
        
        self.gimbal_thrust_line, = ax.plot(
            [engine_point[0], thrust_endpoint[0]],
            [engine_point[1], thrust_endpoint[1]],
            [engine_point[2], thrust_endpoint[2]],
            color=color, linewidth=4, label=f'Thrust ({thrust*100:.0f}%)'
        )
        
        # Draw CG point
        self.cg_point, = ax.plot([0], [0], [0], 'k*', markersize=15, label='Center of Mass')
        
        # Draw coordinate axes
        axis_length = 5
        axes_colors = {'x': 'r', 'y': 'g', 'z': 'b'}
        axes_dirs = {
            'x': geom['rotation_matrix'][:, 0],
            'y': geom['rotation_matrix'][:, 1],
            'z': geom['rotation_matrix'][:, 2]
        }
        
        for axis_name, color in axes_colors.items():
            direction = axes_dirs[axis_name]
            endpoint = direction * axis_length
            line, = ax.plot(
                [0, endpoint[0]],
                [0, endpoint[1]],
                [0, endpoint[2]],
                color=color, linewidth=2, alpha=0.5, label=f'{axis_name}-axis'
            )
            self.coordinate_axes[axis_name] = line
        
        # Add gimbal angle info
        info_text = f'Gimbal: Pitch={np.degrees(gimbal_pitch):.1f}°, Yaw={np.degrees(gimbal_yaw):.1f}°\nThrust={thrust*100:.1f}%'
        ax.text2D(0.05, 0.95, info_text, transform=ax.transAxes, 
                 fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        ax.legend(loc='upper left', fontsize=8)
    
    def _draw_trajectory(self, current_pos):
        """
        Draw trajectory history in the full trajectory view.
        Shows both desired (dotted) and actual (solid) paths.
        
        Args:
            current_pos: Current position [x, y, z]
        """
        ax = self.ax_trajectory
        
        # Clear previous trajectory elements safely
        if self.trajectory_line is not None:
            try:
                self.trajectory_line.remove()
            except:
                pass
            self.trajectory_line = None
            
        if self.desired_trajectory_line is not None:
            try:
                self.desired_trajectory_line.remove()
            except:
                pass
            self.desired_trajectory_line = None
            
        if self.trajectory_current_pos is not None:
            try:
                self.trajectory_current_pos.remove()
            except:
                pass
            self.trajectory_current_pos = None
        
        # Draw actual trajectory (solid green line)
        if len(self.trajectory_history) > 0:
            traj = np.array(self.trajectory_history)
            self.trajectory_line, = ax.plot(
                traj[:, 0], traj[:, 1], traj[:, 2],
                'g-', linewidth=2.5, alpha=0.8, label='Actual Path'
            )
        
        # Draw desired trajectory (dotted blue line)
        if len(self.desired_trajectory_history) > 0:
            desired_traj = np.array(self.desired_trajectory_history)
            self.desired_trajectory_line, = ax.plot(
                desired_traj[:, 0], desired_traj[:, 1], desired_traj[:, 2],
                'b--', linewidth=2, alpha=0.6, label='Target Path'
            )
        
        # Current position marker
        self.trajectory_current_pos, = ax.plot(
            [current_pos[0]], [current_pos[1]], [current_pos[2]],
            'r*', markersize=15, label='Current Position'
        )
        
        # Draw start point only once (never remove it)
        if len(self.trajectory_history) > 0 and self.trajectory_launch_point is None:
            start_pos = self.trajectory_history[0]
            self.trajectory_launch_point, = ax.plot([start_pos[0]], [start_pos[1]], [start_pos[2]], 'go', markersize=8, label='Launch Point')
        
        # Auto-adjust limits based on both actual and desired trajectory
        all_positions = []
        if len(self.trajectory_history) > 0:
            all_positions.extend(self.trajectory_history)
        if len(self.desired_trajectory_history) > 0:
            all_positions.extend(self.desired_trajectory_history)
        
        if len(all_positions) > 10:
            all_pos_array = np.array(all_positions)
            margin = 1000.0  # 1km margin
            
            ax.set_xlim([np.min(all_pos_array[:, 0]) - margin, np.max(all_pos_array[:, 0]) + margin])
            ax.set_ylim([np.min(all_pos_array[:, 1]) - margin, np.max(all_pos_array[:, 1]) + margin])
            ax.set_zlim([np.min(all_pos_array[:, 2]) - margin, max(np.max(all_pos_array[:, 2]) + margin, 1000)])
        
        ax.legend(loc='upper left', fontsize=8)
    
    def update(self, rocket_state, current_time, thrust, gimbal_pitch, gimbal_yaw, desired_position=None):
        """
        Update visualization for current rocket state.
        
        Args:
            rocket_state: RocketState object
            current_time: Current simulation time (seconds)
            thrust: Normalized thrust (0-1)
            gimbal_pitch: Gimbal pitch angle (radians)
            gimbal_yaw: Gimbal yaw angle (radians)
            desired_position: Desired position vector (optional)
        """
        # Update trajectory history
        self.trajectory_history.append(rocket_state.position.copy())
        
        # Update desired trajectory history if provided
        if desired_position is not None:
            self.desired_trajectory_history.append(np.array(desired_position).copy())
        
        self.thrust_history.append(thrust)
        self.gimbal_angle_history.append((gimbal_pitch, gimbal_yaw))
        
        # Draw rocket
        self._draw_rocket_body(rocket_state, thrust, gimbal_pitch, gimbal_yaw)
        
        # Draw trajectory
        self._draw_trajectory(rocket_state.position)
        
        # Rotate both views
        self.rotation_angle += VisualizationParams.rotation_speed * 0.016  # Assuming ~60Hz update
        
        # Keep angle in reasonable range
        self.rotation_angle = self.rotation_angle % 360
        
        # Apply rotation to both plots
        self.ax_rocket.view_init(elev=20, azim=self.rotation_angle)
        self.ax_trajectory.view_init(elev=20, azim=self.rotation_angle)
        
        # Add time info
        time_text = f'Time: {current_time:.2f}s | Altitude: {rocket_state.position[2]:.1f}m | Velocity: {np.linalg.norm(rocket_state.velocity):.1f}m/s'
        self.fig.text(0.5, 0.02, time_text, ha='center', fontsize=11, fontweight='bold')
        
        # Skip tight_layout to avoid issues
        # plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        
        plt.pause(0.001)  # Small pause to allow plot updates
    
    def show(self):
        """Display the visualization."""
        plt.show()
