"""
Interactive 3D Visualization with Playback Controls
Shows pre-computed trajectory with play/pause and speed controls
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from src.config import RocketSpecs, VisualizationParams


class InteractiveRocket3DVisualizer:
    """Interactive 3D visualization with playback controls."""
    
    def __init__(self, telemetry, target_telemetry=None, specs=None, figsize=(16, 8)):
        """
        Initialize interactive visualizer with pre-computed telemetry.
        
        Args:
            telemetry: Dictionary with 'time', 'position', 'velocity', 'thrust', etc.
            target_telemetry: Optional dictionary with desired trajectory
            specs: RocketSpecs object
        """
        self.specs = specs or RocketSpecs()
        self.telemetry = telemetry
        self.target_telemetry = target_telemetry
        self.num_frames = len(telemetry['time'])
        self.current_frame = 0
        self.is_playing = False
        self.speed_multiplier = 1.0
        
        # Create figure with 3D subplots
        self.fig = plt.figure(figsize=figsize)
        self.fig.suptitle('Rocket Control Simulation - Interactive Playback', fontsize=16, fontweight='bold')
        
        # Left plot: Zoomed rocket view
        self.ax_rocket = self.fig.add_subplot(121, projection='3d')
        
        # Right plot: Full trajectory view with fixed target path
        self.ax_trajectory = self.fig.add_subplot(122, projection='3d')
        
        # Setup plots
        self._setup_plots()
        
        # Pre-compute target trajectory (fixed 60km vertical line)
        # self._compute_target_trajectory()
        
        # Rotation angle
        self.rotation_angle = 0.0
        
        # Create control buttons and slider
        self._create_controls()
        
        # Current plot handles
        self.rocket_body_line = None
        self.gimbal_thrust_line = None
        self.cg_point = None
        self.actual_path_line = None
        self.target_path_line = None
        self.current_pos_marker = None
        self.coordinate_axes = {}
        self.info_text = None
        self._colorbar = None  # Colorbar for trajectory heatmap
        
        # Animation timer - setup ONCE on initialization
        self.timer = self.fig.canvas.new_timer(interval=50)  # 50ms = 20fps
        self.timer.single_shot = False
        self.timer.callbacks.append((self._animate_frame, (), {}))
        self.timer.start()  # Start immediately, controlled by is_playing flag
        
        # Flag to ignore slider callbacks during programmatic updates
        self._updating_slider_programmatically = False
    
    def _setup_plots(self):
        """Setup the two 3D plot areas."""
        # Rocket detail view
        ax = self.ax_rocket
        ax.set_xlabel('X (m)', fontweight='bold')
        ax.set_ylabel('Y (m)', fontweight='bold')
        ax.set_zlabel('Z (m)', fontweight='bold')
        ax.set_title('Rocket Detail View (CG at Origin)', fontsize=12, fontweight='bold')
        limit = 15
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.set_zlim([-limit, limit])
        ax.grid(True, alpha=0.3)
        ax.set_box_aspect([1, 1, 1])
        # Set viewing angle: elevation=20, azimuth=45 (Z visible as vertical)
        ax.view_init(elev=20, azim=45)
        
        # Trajectory view
        ax = self.ax_trajectory
        ax.set_xlabel('X (m)', fontweight='bold')
        ax.set_ylabel('Y (m)', fontweight='bold')
        ax.set_zlabel('Z (m)', fontweight='bold')
        ax.set_title('Trajectory View (Target & Actual)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_box_aspect([1, 1, 1])
        # Set viewing angle: elevation=20, azimuth=45 (Z visible as vertical)
        ax.view_init(elev=20, azim=45)
    
    def _compute_target_trajectory(self):
        """Pre-compute the target trajectory from telemetry or use default."""
        if self.target_telemetry is not None:
            # Use provided target trajectory
            self.target_trajectory = self.target_telemetry['position']
        else:
            # Default: straight vertical line to 60km
            target_altitude = 60_000.0
            num_points = 1000
            self.target_trajectory = np.array([
                [0.0, 0.0, z] for z in np.linspace(0, target_altitude, num_points)
            ])
    
    def _create_controls(self):
        """Create playback control buttons and slider."""
        # Adjust layout to make room for controls
        plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.35)
        
        # Slider for frame position
        ax_slider = plt.axes([0.15, 0.10, 0.7, 0.03])
        self.slider = Slider(
            ax_slider, 'Frame', 0, self.num_frames - 1,
            valinit=0, valstep=1, color='steelblue'
        )
        self.slider.on_changed(self._on_slider_changed)
        
        # Play/Pause button
        ax_play = plt.axes([0.15, 0.20, 0.06, 0.04])
        self.btn_play = Button(ax_play, 'Play', hovercolor='0.975')
        self.btn_play.on_clicked(self._on_play_clicked)
        
        # Speed buttons - store references so they don't get garbage collected
        self.speed_buttons = []
        speeds = [1, 2, 4, 8, 16, 32, 64]
        for i, speed in enumerate(speeds):
            ax = plt.axes([0.23 + i*0.07, 0.20, 0.06, 0.04])
            btn = Button(ax, f'{speed}x', hovercolor='0.975')
            # Store button reference
            self.speed_buttons.append(btn)
            # Create closure properly to capture speed value
            def make_callback(s):
                def callback(evt):
                    self._on_speed_clicked(s)
                return callback
            btn.on_clicked(make_callback(speed))
    
    def _on_slider_changed(self, val):
        """Handle slider position change."""
        # Ignore if this is a programmatic update during playback
        if self._updating_slider_programmatically:
            return
        
        # User manually moved slider - stop playback
        self.current_frame = int(val)
        self.is_playing = False
        self.btn_play.label.set_text('Play')
        self._update_display()
    
    def _on_play_clicked(self, event):
        """Handle play/pause button."""
        self.is_playing = not self.is_playing
        self.btn_play.label.set_text('Pause' if self.is_playing else 'Play')
    
    def _on_speed_clicked(self, speed):
        """Handle speed button click."""
        self.speed_multiplier = speed
        # Update display immediately to show speed change
        self._update_info_text()
    
    def _animate_frame(self):
        """Single animation frame callback - runs every 50ms via timer."""
        if self.is_playing and self.current_frame < self.num_frames - 1:
            # Advance frame based on speed multiplier
            # At 20fps (50ms), 1x speed = 1 frame/tick, 2x = 2 frames/tick, etc
            self.current_frame = min(
                self.current_frame + max(1, int(self.speed_multiplier)),
                self.num_frames - 1
            )
            
            # Stop at end
            if self.current_frame >= self.num_frames - 1:
                self.current_frame = self.num_frames - 1
                self.is_playing = False
                self.btn_play.label.set_text('Play')
            
            # Update slider WITHOUT triggering _on_slider_changed callback
            self._updating_slider_programmatically = True
            self.slider.set_val(self.current_frame)
            self._updating_slider_programmatically = False
            
            # Update display
            self._update_display()
    
    def _update_display(self):
        """Update the visualization for current frame."""
        frame = self.current_frame
        
        # Get current state
        time = self.telemetry['time'][frame]
        position = np.array(self.telemetry['position'][frame])
        thrust = self.telemetry['thrust'][frame]
        gimbal_pitch = self.telemetry['gimbal_pitch'][frame]
        gimbal_yaw = self.telemetry['gimbal_yaw'][frame]
        
        # Get orientation - use actual orientation if available, otherwise approximate from gimbals
        if 'orientation' in self.telemetry and len(self.telemetry['orientation']) > 0:
            orientation = np.array(self.telemetry['orientation'][frame])
        else:
            # Fallback: approximate from gimbal angles
            orientation = np.array([0.0, gimbal_pitch, gimbal_yaw])
        
        # Update rocket view
        self._draw_rocket(position, orientation, thrust, gimbal_pitch, gimbal_yaw)
        
        # Get desired position for visualization
        desired_position = np.array(self.telemetry['desired_position'][frame]) if 'desired_position' in self.telemetry else position
        
        # Update trajectory view
        self._draw_trajectory(frame, position, desired_position)
        
        # Rotate both views
        self.rotation_angle += VisualizationParams.rotation_speed * 0.016
        self.rotation_angle = self.rotation_angle % 360
        
        self.ax_rocket.view_init(elev=20, azim=self.rotation_angle)
        self.ax_trajectory.view_init(elev=20, azim=self.rotation_angle)
        
        # Update title with time info (remove old text first)
        if self.info_text is not None:
            try:
                self.info_text.remove()
            except:
                pass
        
        info_text = f'Time: {time:.2f}s | Altitude: {position[2]:.1f}m | Velocity: {np.linalg.norm(self.telemetry["velocity"][frame]):.1f}m/s | Speed: {self.speed_multiplier}x'
        self.info_text = self.fig.text(0.5, 0.12, info_text, ha='center', fontsize=11, fontweight='bold',
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        self.fig.canvas.draw_idle()
    
    def _update_info_text(self):
        """Update only the info text display (for quick updates like speed changes)."""
        if not hasattr(self, 'current_frame') or not hasattr(self, 'telemetry'):
            return
        
        frame = self.current_frame
        time = self.telemetry['time'][frame]
        position = np.array(self.telemetry['position'][frame])
        
        if self.info_text is not None:
            try:
                self.info_text.remove()
            except:
                pass
        
        info_text = f'Time: {time:.2f}s | Altitude: {position[2]:.1f}m | Velocity: {np.linalg.norm(self.telemetry["velocity"][frame]):.1f}m/s | Speed: {self.speed_multiplier}x'
        self.info_text = self.fig.text(0.5, 0.12, info_text, ha='center', fontsize=11, fontweight='bold',
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        self.fig.canvas.draw_idle()
    
    def _draw_rocket(self, position, orientation, thrust, gimbal_pitch, gimbal_yaw):
        """Draw the rocket in the detail view."""
        ax = self.ax_rocket
        
        # Clear previous elements
        if self.rocket_body_line is not None:
            try:
                self.rocket_body_line.remove()
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
        
        # Build rotation matrix
        alpha1, alpha2, alpha3 = orientation
        c1, s1 = np.cos(alpha1), np.sin(alpha1)
        c2, s2 = np.cos(alpha2), np.sin(alpha2)
        c3, s3 = np.cos(alpha3), np.sin(alpha3)
        
        R = np.array([
            [c3*c2, c3*s2*s1 - s3*c1, c3*s2*c1 + s3*s1],
            [s3*c2, s3*s2*s1 + c3*c1, s3*s2*c1 - c3*s1],
            [-s2,   c2*s1,            c2*c1]
        ])
        
        # Rocket body
        body_length = self.specs.length
        cg_offset = self.specs.engine_offset_from_cg
        
        nose_tip_body = np.array([0.0, 0.0, body_length / 2])
        engine_point_body = np.array([0.0, 0.0, -cg_offset])
        rocket_base_body = np.array([0.0, 0.0, -body_length / 2])
        
        nose_tip = R @ nose_tip_body
        engine_point = R @ engine_point_body
        rocket_base = R @ rocket_base_body
        
        # Draw rocket body
        self.rocket_body_line, = ax.plot(
            [rocket_base[0], nose_tip[0]],
            [rocket_base[1], nose_tip[1]],
            [rocket_base[2], nose_tip[2]],
            'b-', linewidth=3, label='Rocket Body'
        )
        
        # Draw gimbal thrust vector (points downward from engine)
        thrust_length = 1.0 + thrust * 9.0
        thrust_dir_body = np.array([
            np.sin(gimbal_yaw),
            np.sin(gimbal_pitch),
            -np.cos(gimbal_pitch) * np.cos(gimbal_yaw)  # NEGATIVE: thrust points downward
        ])
        thrust_dir_body = thrust_dir_body / np.linalg.norm(thrust_dir_body) * thrust_length
        thrust_dir_inertial = R @ thrust_dir_body
        thrust_endpoint = engine_point + thrust_dir_inertial
        
        # Color based on thrust
        color = [
            (1 - thrust) * 0 + thrust * 1,  # Red channel
            (1 - thrust) * 1 + thrust * 0,  # Green channel
            0  # Blue channel
        ]
        
        self.gimbal_thrust_line, = ax.plot(
            [engine_point[0], thrust_endpoint[0]],
            [engine_point[1], thrust_endpoint[1]],
            [engine_point[2], thrust_endpoint[2]],
            color=color, linewidth=4, label=f'Thrust ({thrust*100:.0f}%)'
        )
        
        # Draw CG
        self.cg_point, = ax.plot([0], [0], [0], 'k*', markersize=15, label='CG')
        
        # Coordinate axes
        axis_length = 5
        axes_colors = {'x': 'r', 'y': 'g', 'z': 'b'}
        axes_dirs = {
            'x': R[:, 0],
            'y': R[:, 1],
            'z': R[:, 2]
        }
        
        for axis_name, color in axes_colors.items():
            direction = axes_dirs[axis_name]
            endpoint = direction * axis_length
            line, = ax.plot([0, endpoint[0]], [0, endpoint[1]], [0, endpoint[2]],
                           color=color, linewidth=2, alpha=0.5, label=f'{axis_name}-axis')
            self.coordinate_axes[axis_name] = line
        
        ax.legend(loc='upper left', fontsize=8)
    def _draw_trajectory(self, frame, current_pos, desired_pos):
        """Draw actual trajectory with heatmap coloring by time."""
        ax = self.ax_trajectory
        
        # Draw full actual trajectory (ONCE) with heatmap coloring by time
        if self.actual_path_line is None and len(self.telemetry['position']) > 1:
            actual_traj = np.array(self.telemetry['position'])
            times = np.array(self.telemetry['time'])
            
            # Scatter plot colored by time (heatmap)
            scatter = ax.scatter(
                actual_traj[:, 0],
                actual_traj[:, 1],
                actual_traj[:, 2],
                c=times,
                cmap='hot',
                s=5,
                alpha=0.7,
                label='Actual Path (colored by time)',
                zorder=5
            )
            
            # Also add continuous line through all points
            self.actual_path_line, = ax.plot(
                actual_traj[:, 0],
                actual_traj[:, 1],
                actual_traj[:, 2],
                'orange', linewidth=1.0, alpha=0.3, zorder=4
            )
            
            # Add colorbar showing time progression
            self._colorbar = plt.colorbar(scatter, ax=ax, label='Time (s)', pad=0.1, shrink=0.8)
        
        # Draw desired trajectory as smooth line (NOT pre-sampled - compute it fresh)
        if self.target_path_line is None:
            # Sample the desired path at dense points
            times_dense = np.linspace(0, self.telemetry['time'][-1], 1000)
            desired_positions = []
            
            # Import trajectory object - we need it to compute desired positions
            # For now, just draw a reference line showing the full recorded desired path
            if 'desired_position' in self.telemetry:
                desired_traj = np.array(self.telemetry['desired_position'])
                self.target_path_line, = ax.plot(
                    desired_traj[:, 0],
                    desired_traj[:, 1],
                    desired_traj[:, 2],
                    'b-', linewidth=2.5, alpha=0.8, label='Desired Path', zorder=10
                )
        
        # Update current position marker (dynamic)
        if self.current_pos_marker is not None:
            try:
                self.current_pos_marker.remove()
            except:
                pass
        
        # Draw actual position (yellow star)
        self.current_pos_marker = ax.scatter(
            [current_pos[0]], [current_pos[1]], [current_pos[2]],
            c='yellow', s=200, marker='*', 
            edgecolors='orange', linewidths=2,
            label='Actual Position', zorder=20
        )
        
        # Draw desired position (cyan square)
        if hasattr(self, '_desired_pos_marker') and self._desired_pos_marker is not None:
            try:
                self._desired_pos_marker.remove()
            except:
                pass
        
        self._desired_pos_marker = ax.scatter(
            [desired_pos[0]], [desired_pos[1]], [desired_pos[2]],
            c='cyan', s=150, marker='s',
            edgecolors='blue', linewidths=2, alpha=0.7,
            label='Desired Position', zorder=19
        )
        
        # Auto-scale
        if frame > 100:
            actual_traj = np.array(self.telemetry['position'][:frame+1])
            margin = 2000
            ax.set_xlim([-margin, margin])
            ax.set_ylim([-margin, margin])
            ax.set_zlim([0, max(62000, np.max(actual_traj[:, 2]) + 2000)])
        else:
            ax.set_xlim([-5000, 5000])
            ax.set_ylim([-5000, 5000])
            ax.set_zlim([0, 65000])
        
        ax.legend(loc='upper left', fontsize=9, framealpha=0.95)
    
    def show(self):
        """Show the interactive visualization."""
        plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.25)
        plt.show()
