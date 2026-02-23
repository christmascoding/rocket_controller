import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
from src.config import CONFIG


def _rotation_matrix_from_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a / max(np.linalg.norm(a), 1e-9)
    b = b / max(np.linalg.norm(b), 1e-9)
    v = np.cross(a, b)
    c = np.dot(a, b)
    if c < -0.999999:
        # Opposite direction, rotate 180 degrees around any orthogonal axis
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / max(np.linalg.norm(v), 1e-9)
        H = np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ]
        )
        return -np.eye(3) + 2.0 * np.outer(v, v)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    v = v / s
    vx = np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )
    R = np.eye(3) + vx * s + (vx @ vx) * (1.0 - c)
    return R


def _make_cylinder(radius: float, z0: float, z1: float, n: int = 8):
    theta = np.linspace(0, 2 * np.pi, n)
    z = np.linspace(z0, z1, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x = radius * np.cos(theta_grid)
    y = radius * np.sin(theta_grid)
    return x, y, z_grid


def _make_cone(radius: float, z0: float, z1: float, n: int = 8):
    theta = np.linspace(0, 2 * np.pi, n)
    z = np.linspace(z0, z1, 2)
    r = np.linspace(radius, 0.0, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    r_grid, _ = np.meshgrid(r, theta)
    r_grid = r_grid.T
    x = r_grid * np.cos(theta_grid)
    y = r_grid * np.sin(theta_grid)
    return x, y, z_grid


def _direction_from_pitch_yaw(pitch: float, yaw: float) -> np.ndarray:
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    return np.array([cy * sp, sy * sp, cp], dtype=float)


def _rotation_matrix_from_pitch_yaw(pitch: float, yaw: float) -> np.ndarray:
    """Create rotation matrix from pitch and yaw angles"""
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    return np.array(
        [
            [cy * cp, -sy, cy * sp],
            [sy * cp, cy, sy * sp],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )


def _get_thrust_color(thrust_pct: float) -> str:
    """Get color based on thrust percentage (0-1)"""
    if thrust_pct > 0.8:
        return "#FF4500"  # OrangeRed
    elif thrust_pct > 0.4:
        return "#FFA500"  # Orange
    else:
        return "#FFD700"  # Gold


def _downsample_trail(data: np.ndarray, stride: int) -> np.ndarray:
    """Downsample trail data for performance"""
    if len(data) <= stride:
        return data
    return data[::stride]


def _set_equal_axes(ax, ref_pos: np.ndarray, margin_ratio: float = 0.2):
    """Set 3D axes with proper bounds to show entire trajectory"""
    finite = np.isfinite(ref_pos).all(axis=1)
    ref = ref_pos[finite]
    if ref.size == 0:
        return
    min_vals = ref.min(axis=0)
    max_vals = ref.max(axis=0)
    ranges = max_vals - min_vals
    margins = ranges * margin_ratio + 1.0  # Add 1m minimum margin
    
    ax.set_xlim(min_vals[0] - margins[0], max_vals[0] + margins[0])
    ax.set_ylim(min_vals[1] - margins[1], max_vals[1] + margins[1])
    ax.set_zlim(min_vals[2] - margins[2], max_vals[2] + margins[2])
    
    # Ensure minimum range to prevent singular matrix errors
    ranges = np.maximum(ranges, 1.0)
    max_range = ranges.max()
    if max_range > 0:
        ax.set_box_aspect(ranges / max_range)


def animate_trajectory(history: dict, trail_length: int = 500):
    """
    Animate rocket trajectory with three-panel layout:
    - Left: Rocket close-up (attitude/gimbal view)
    - Center: Main trajectory view
    - Right: MPC internals (3 stacked plots)
    """
    # Extract data from history
    pos = history["position"]
    pos_ref = history["pos_ref"]
    pos_closest = history["pos_closest"]
    gimbal = history["gimbal_angles"]
    thrust = history["thrust_n"]
    velocity = history["velocity"]
    attitude = history["attitude"]
    time = history["time"]
    
    # MPC data
    mpc_pred_pos = history["mpc_pred_pos"]
    mpc_ref_pos = history["mpc_ref_pos"]
    mpc_cost_pos = history["mpc_cost_pos"]
    mpc_cost_vel = history["mpc_cost_vel"]
    mpc_cost_accel = history["mpc_cost_accel"]
    mpc_cost_jerk = history["mpc_cost_jerk"]
    mpc_cost_total = history["mpc_cost_total"]
    
    max_thrust = thrust.max() if thrust.max() > 1e-6 else 1.0
    
    # Apply update interval for performance
    update_interval = CONFIG.viz.update_interval
    num_frames = len(pos) // update_interval
    
    # ===== FIGURE SETUP =====
    fig = plt.figure(figsize=(22, 10))
    # 4 columns: left 3D, center 3D, right top-left plot, right top-right plot
    # Heights match to make right plots square
    gs = fig.add_gridspec(2, 4, width_ratios=[1.0, 1.4, 1.0, 1.0], height_ratios=[1.0, 1.0],
                          hspace=0.3, wspace=0.5)
    
    # ===== LEFT PANEL: Rocket Close-Up =====
    ax_closeup = fig.add_subplot(gs[:, 0], projection="3d")
    ax_closeup.set_title("Rocket Attitude & Thrust", fontsize=12, fontweight='bold')
    ax_closeup.set_xlabel("X [m]")
    ax_closeup.set_ylabel("Y [m]")
    ax_closeup.set_zlabel("Z [m]")
    ax_closeup.set_xlim(-10, 10)
    ax_closeup.set_ylim(-10, 10)
    ax_closeup.set_zlim(0, 15)
    ax_closeup.view_init(elev=20, azim=45)
    ax_closeup.set_box_aspect((1, 1, 1.5))
    
    # Disable mouse interaction for close-up view
    ax_closeup.disable_mouse_rotation()
    
    # Disable coordinate formatting to prevent transformation matrix errors
    ax_closeup.format_coord = lambda x, y: ""
    
    # Show all three axes with visible panes for better navigation
    ax_closeup.xaxis.pane.fill = True
    ax_closeup.yaxis.pane.fill = True
    ax_closeup.zaxis.pane.fill = True
    ax_closeup.xaxis.pane.set_edgecolor('black')
    ax_closeup.yaxis.pane.set_edgecolor('black')
    ax_closeup.zaxis.pane.set_edgecolor('black')
    ax_closeup.xaxis.pane.set_alpha(0.2)
    ax_closeup.yaxis.pane.set_alpha(0.2)
    ax_closeup.zaxis.pane.set_alpha(0.2)
    ax_closeup.grid(True, alpha=0.3)
    
    # Force all three axes to always be visible (override matplotlib 3D defaults)
    ax_closeup.xaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    ax_closeup.yaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    ax_closeup.zaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    # Draw all axis lines regardless of view angle
    ax_closeup.xaxis.line.set_linewidth(1.5)
    ax_closeup.yaxis.line.set_linewidth(1.5)
    ax_closeup.zaxis.line.set_linewidth(1.5)
    # Make labels always visible
    ax_closeup.xaxis.label.set_visible(True)
    ax_closeup.yaxis.label.set_visible(True)
    ax_closeup.zaxis.label.set_visible(True)
    
    # Coordinate frame triads at origin
    ax_closeup.quiver(0, 0, 0, 2, 0, 0, color='r', linewidth=1.5, arrow_length_ratio=0.2)
    ax_closeup.quiver(0, 0, 0, 0, 2, 0, color='g', linewidth=1.5, arrow_length_ratio=0.2)
    ax_closeup.quiver(0, 0, 0, 0, 0, 2, color='b', linewidth=1.5, arrow_length_ratio=0.2)
    
    # Body Z-axis reference (dashed green line)
    ax_closeup.plot([0, 0], [0, 0], [0, 8], 'g--', linewidth=1.5, alpha=0.5)
    
    # Close-up elements (will be updated)
    closeup_body = None
    closeup_cone = None
    closeup_thrust_stick, = ax_closeup.plot([], [], [], linewidth=5, color='orange')
    closeup_gimbal_cone = None
    closeup_text = ax_closeup.text2D(0.02, 0.98, "", transform=ax_closeup.transAxes,
                                      fontsize=9, verticalalignment='top',
                                      family='monospace',
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # ===== CENTER PANEL: Main Trajectory =====
    ax_main = fig.add_subplot(gs[:, 1], projection="3d")
    ax_main.set_title("Rocket Trajectory", fontsize=12, fontweight='bold')
    ax_main.set_xlabel("X [m]")
    ax_main.set_ylabel("Y [m]")
    ax_main.set_zlabel("Z [m]")
    
    # Disable coordinate formatting to prevent transformation matrix errors
    ax_main.format_coord = lambda x, y: ""
    
    # Show all three axes with visible panes for better navigation
    ax_main.xaxis.pane.fill = True
    ax_main.yaxis.pane.fill = True
    ax_main.zaxis.pane.fill = True
    ax_main.xaxis.pane.set_edgecolor('black')
    ax_main.yaxis.pane.set_edgecolor('black')
    ax_main.zaxis.pane.set_edgecolor('black')
    ax_main.xaxis.pane.set_alpha(0.2)
    ax_main.yaxis.pane.set_alpha(0.2)
    ax_main.zaxis.pane.set_alpha(0.2)
    ax_main.grid(True, alpha=0.3)
    
    # Force all three axes to always be visible (override matplotlib 3D defaults)
    ax_main.xaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    ax_main.yaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    ax_main.zaxis.set_pane_color((0.9, 0.9, 0.9, 0.3))
    # Draw all axis lines regardless of view angle
    ax_main.xaxis.line.set_linewidth(1.5)
    ax_main.yaxis.line.set_linewidth(1.5)
    ax_main.zaxis.line.set_linewidth(1.5)
    # Make labels always visible
    ax_main.xaxis.label.set_visible(True)
    ax_main.yaxis.label.set_visible(True)
    ax_main.zaxis.label.set_visible(True)
    
    # Plot full trajectory path (static)
    ax_main.plot(pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2], "--", 
                 color="green", alpha=0.5, linewidth=1, label="Target Path")
    
    # Dynamic elements
    main_trail, = ax_main.plot([], [], [], color="orange", linewidth=2, label="Actual Path")
    main_rocket_dot, = ax_main.plot([], [], [], "o", color="red", markersize=8)
    main_closest_dot, = ax_main.plot([], [], [], "o", color="purple", markersize=6, alpha=0.7)
    main_gimbal_vec = ax_main.quiver(0, 0, 0, 0, 0, 1, length=5.0, color="cyan", linewidth=2)
    main_body = None
    main_cone = None
    main_stick, = ax_main.plot([], [], [], color="orange", linewidth=3)
    
    ax_main.legend(loc='upper right', fontsize=9)
    
    # Set axes to include both reference path AND actual trajectory
    all_positions = np.vstack([pos_ref, pos])
    _set_equal_axes(ax_main, all_positions, margin_ratio=0.2)
    
    # Set better 3D view angle for trajectory (isometric-ish view)
    ax_main.view_init(elev=30, azim=45)
    
    # ===== RIGHT PANEL: MPC Internals (2x2 Grid) =====
    # Use direct subplot positions instead of subgridspec to avoid squeezing
    
    # Top-left: Position Error
    ax_error = fig.add_subplot(gs[0, 2])
    ax_error.set_title("Position Error", fontsize=10, fontweight='bold')
    ax_error.set_xlabel("Time [s]", fontsize=9)
    ax_error.set_ylabel("Error [m]", fontsize=9)
    ax_error.grid(True, alpha=0.3)
    pos_error = np.linalg.norm(pos - pos_closest, axis=1)
    error_line, = ax_error.plot([], [], color="red", linewidth=1.5)
    error_fill = None
    error_marker = ax_error.axvline(time[0], color="black", linestyle="--", linewidth=1, alpha=0.5)
    
    # Top-right: MPC Horizon Projection
    ax_horizon = fig.add_subplot(gs[0, 3])
    if CONFIG.viz.mpc_horizon_projection == "xy":
        ax_horizon.set_title("MPC Horizon (Top View)", fontsize=10, fontweight='bold')
        ax_horizon.set_xlabel("X [m]", fontsize=9)
        ax_horizon.set_ylabel("Y [m]", fontsize=9)
    else:  # xz
        ax_horizon.set_title("MPC Horizon (Side View)", fontsize=10, fontweight='bold')
        ax_horizon.set_xlabel("X [m]", fontsize=9)
        ax_horizon.set_ylabel("Z [m]", fontsize=9)
    ax_horizon.grid(True, alpha=0.3)
    # Remove equal aspect to prevent squeezing
    # ax_horizon.set_aspect('equal', adjustable='box')
    
    horizon_current_dot, = ax_horizon.plot([], [], 'ro', markersize=8, label='Current', zorder=5)
    horizon_ref_line, = ax_horizon.plot([], [], 'g-o', markersize=4, linewidth=1.5,
                                        alpha=0.7, label='Reference', zorder=3)
    horizon_closest_dot, = ax_horizon.plot([], [], 'mo', markersize=6, label='Closest', zorder=4)
    horizon_path_line, = ax_horizon.plot([], [], color='gray', linewidth=1, 
                                         alpha=0.3, linestyle='--', zorder=1)
    ax_horizon.legend(loc='best', fontsize=8)
    
    # Bottom-left: MPC Cost Components
    ax_cost = fig.add_subplot(gs[1, 2])
    ax_cost.set_title("MPC Cost Components", fontsize=10, fontweight='bold')
    ax_cost.set_xlabel("Time [s]", fontsize=9)
    ax_cost.set_ylabel("Cost [-]", fontsize=9)
    ax_cost.grid(True, alpha=0.3)
    
    cost_pos_line, = ax_cost.plot([], [], color="blue", linewidth=1.5, label="Position", alpha=0.8)
    cost_vel_line, = ax_cost.plot([], [], color="green", linewidth=1.5, label="Velocity", alpha=0.8)
    cost_accel_line, = ax_cost.plot([], [], color="orange", linewidth=1.5, label="Accel", alpha=0.8)
    cost_jerk_line, = ax_cost.plot([], [], color="red", linewidth=1.5, label="Jerk", alpha=0.8)
    cost_total_line, = ax_cost.plot([], [], color="black", linewidth=2, label="Total")
    cost_marker = ax_cost.axvline(time[0], color="black", linestyle="--", linewidth=1, alpha=0.5)
    ax_cost.legend(loc='upper right', fontsize=8, ncol=2)
    
    # Bottom-right: Reserved for future use
    ax_future = fig.add_subplot(gs[1, 3])
    ax_future.set_title("Reserved", fontsize=10, fontweight='bold')
    ax_future.text(0.5, 0.5, "Available for\nfuture plots", 
                   ha='center', va='center', fontsize=12, color='gray',
                   transform=ax_future.transAxes)
    ax_future.set_xticks([])
    ax_future.set_yticks([])
    ax_future.spines['top'].set_visible(False)
    ax_future.spines['right'].set_visible(False)
    ax_future.spines['bottom'].set_visible(False)
    ax_future.spines['left'].set_visible(False)
    
    # ===== TIMELINE SEEKBAR =====
    # Timeline slider at top (spans most of width)
    ax_timeline = fig.add_axes([0.1, 0.975, 0.55, 0.015])
    slider_timeline = Slider(
        ax_timeline, 'Frame', 0, num_frames - 1, 
        valinit=0, valstep=1, color='steelblue'
    )
    
    # Store current frame and playback speed
    current_frame_ref = [0]  # Use list for mutable reference in nested function
    playback_speed = [1]     # Multiplier for animation speed
    user_seeking = [False]   # Flag to indicate if user is actively dragging slider
    is_playing = [True]      # Animation play state
    
    def _seek_frame(val):
        """Callback for timeline slider - seek to specific frame"""
        user_seeking[0] = True
        current_frame_ref[0] = int(val)
        # Force update without waiting for animation tick
        fig.canvas.draw_idle()
    
    slider_timeline.on_changed(_seek_frame)
    
    # ===== SPEED CONTROL BUTTONS =====
    ax_btn_1x = fig.add_axes([0.68, 0.96, 0.03, 0.03])
    ax_btn_2x = fig.add_axes([0.72, 0.96, 0.03, 0.03])
    ax_btn_4x = fig.add_axes([0.76, 0.96, 0.03, 0.03])
    ax_btn_8x = fig.add_axes([0.80, 0.96, 0.03, 0.03])
    ax_btn_16x = fig.add_axes([0.84, 0.96, 0.03, 0.03])
    ax_btn_32x = fig.add_axes([0.88, 0.96, 0.03, 0.03])
    
    btn_1x = Button(ax_btn_1x, "1x", color='lightblue', hovercolor='skyblue')
    btn_2x = Button(ax_btn_2x, "2x", color='lightgreen', hovercolor='lightgreen')
    btn_4x = Button(ax_btn_4x, "4x", color='lightyellow', hovercolor='lightyellow')
    btn_8x = Button(ax_btn_8x, "8x", color='lightcoral', hovercolor='lightcoral')
    btn_16x = Button(ax_btn_16x, "16x", color='plum', hovercolor='plum')
    btn_32x = Button(ax_btn_32x, "32x", color='lightgray', hovercolor='lightgray')
    
    def _set_speed(speed: int):
        """Set playback speed multiplier and update animation interval"""
        playback_speed[0] = speed
        # Adjust animation speed: lower interval = faster playback
        # Base interval is 30ms, divide by speed multiplier
        anim.event_source.interval = max(1, int(30 / speed))
    
    btn_1x.on_clicked(lambda _event: _set_speed(1))
    btn_2x.on_clicked(lambda _event: _set_speed(2))
    btn_4x.on_clicked(lambda _event: _set_speed(4))
    btn_8x.on_clicked(lambda _event: _set_speed(8))
    btn_16x.on_clicked(lambda _event: _set_speed(16))
    btn_32x.on_clicked(lambda _event: _set_speed(32))
    
    # ===== PLAY/PAUSE BUTTON =====
    ax_btn_play = fig.add_axes([0.65, 0.96, 0.025, 0.03])
    btn_play = Button(ax_btn_play, "Play", color='lightsteelblue', hovercolor='dodgerblue')
    
    def _toggle_playback(_event):
        """Toggle animation play/pause"""
        is_playing[0] = not is_playing[0]
        if is_playing[0]:
            btn_play.label.set_text("Pause")
            anim.resume()
        else:
            btn_play.label.set_text("Play")
            anim.pause()
        fig.canvas.draw_idle()
    
    btn_play.on_clicked(_toggle_playback)
    
    # ===== ANIMATION UPDATE FUNCTION =====
    def update(frame_idx: int):
        nonlocal closeup_body, closeup_cone, closeup_gimbal_cone
        nonlocal main_body, main_cone, main_gimbal_vec
        nonlocal error_fill
        
        # If user is not seeking, increment normally; otherwise use slider value
        if not user_seeking[0]:
            current_frame_ref[0] = frame_idx
        user_seeking[0] = False  # Reset seeking flag for next frame
        
        # Calculate actual index with update_interval downsampling
        display_idx = current_frame_ref[0]
        actual_idx = min(display_idx * update_interval, len(pos) - 1)
        
        # Update slider to match current playback position (from animation)
        if display_idx < num_frames:
            slider_timeline.set_val(display_idx)
        
        # Current state
        p = pos[actual_idx]
        g = gimbal[actual_idx]
        t = thrust[actual_idx]
        att = attitude[actual_idx]
        t_now = time[actual_idx]
        
        # Body rotation matrix
        R_body = _rotation_matrix_from_pitch_yaw(att[1], att[2])
        
        # Gimbal direction in body frame
        gimbal_dir_body = _direction_from_pitch_yaw(g[0], g[1])
        # Gimbal direction in world frame
        gimbal_dir_world = R_body @ gimbal_dir_body
        
        thrust_pct = t / max_thrust
        thrust_color = _get_thrust_color(thrust_pct)
        
        # ===== UPDATE CLOSE-UP VIEW =====
        
        # Remove old surfaces
        if closeup_body is not None:
            try:
                closeup_body.remove()
            except (ValueError, AttributeError):
                pass
        if closeup_cone is not None:
            try:
                closeup_cone.remove()
            except (ValueError, AttributeError):
                pass
        if closeup_gimbal_cone is not None:
            try:
                closeup_gimbal_cone.remove()
            except (ValueError, AttributeError):
                pass
        
        # Rocket body centered at origin (middle of rocket)
        body_length = 6.0
        body_radius = 0.8
        cone_length = 1.0
        total_length = body_length + cone_length  # 7.0m
        center_offset = total_length / 2.0  # 3.5m - offset to center the rocket
        
        # Body from -3.5 to +2.5 (centered at 0)
        z0 = -center_offset
        z1 = z0 + body_length  # -3.5 + 6.0 = 2.5
        # Nose cone from 2.5 to 3.5
        cone_z0 = z1
        cone_z1 = z1 + cone_length  # 2.5 + 1.0 = 3.5
        
        x_cyl, y_cyl, z_cyl = _make_cylinder(body_radius, z0, z1, n=CONFIG.viz.mesh_quality)
        x_cone, y_cone, z_cone = _make_cone(body_radius, cone_z0, cone_z1, n=CONFIG.viz.mesh_quality)
        
        # Rotate by body attitude
        def transform_closeup(x, y, z):
            pts = np.stack([x, y, z], axis=-1)
            shp = pts.shape
            pts_flat = pts.reshape(-1, 3)
            # Translate to have base at z=0, rotate, done (no translation back)
            pts_rotated = pts_flat @ R_body.T
            pts_rotated = pts_rotated.reshape(shp)
            return pts_rotated[..., 0], pts_rotated[..., 1], pts_rotated[..., 2]
        
        x_cyl_r, y_cyl_r, z_cyl_r = transform_closeup(x_cyl, y_cyl, z_cyl)
        x_cone_r, y_cone_r, z_cone_r = transform_closeup(x_cone, y_cone, z_cone)
        
        closeup_body = ax_closeup.plot_surface(x_cyl_r, y_cyl_r, z_cyl_r,
                                                color='lightsteelblue', alpha=0.9, linewidth=0)
        closeup_cone = ax_closeup.plot_surface(x_cone_r, y_cone_r, z_cone_r,
                                                color='silver', alpha=0.9, linewidth=0)
        
        # Thrust stick (from nozzle at z=-3.5 in gimbal direction)
        nozzle_pos = np.array([0, 0, -center_offset])  # Engine at base of rocket
        thrust_stick_len = 8.0 * thrust_pct
        # Rotate nozzle position by body attitude to get world position
        nozzle_pos_world = R_body @ nozzle_pos
        thrust_tip = nozzle_pos_world + gimbal_dir_world * thrust_stick_len
        closeup_thrust_stick.set_data([nozzle_pos_world[0], thrust_tip[0]], 
                                       [nozzle_pos_world[1], thrust_tip[1]])
        closeup_thrust_stick.set_3d_properties([nozzle_pos_world[2], thrust_tip[2]])
        closeup_thrust_stick.set_color(thrust_color)
        
        # Gimbal deflection cone (transparent) - positioned at nozzle
        gimbal_angle_mag = np.linalg.norm(g)
        if gimbal_angle_mag > 0.01:
            cone_height = 3.0
            cone_base_radius = cone_height * np.tan(gimbal_angle_mag)
            x_g, y_g, z_g = _make_cone(cone_base_radius, 0, cone_height, n=CONFIG.viz.mesh_quality)
            
            # Rotate cone to point in gimbal direction and position at nozzle
            R_gimbal = _rotation_matrix_from_vectors(np.array([0, 0, 1]), gimbal_dir_world)
            def transform_gimbal(x, y, z):
                pts = np.stack([x, y, z], axis=-1)
                shp = pts.shape
                pts_flat = pts.reshape(-1, 3) @ R_gimbal.T
                # Translate to nozzle position
                pts_flat += nozzle_pos_world
                return pts_flat.reshape(shp)[..., 0], pts_flat.reshape(shp)[..., 1], pts_flat.reshape(shp)[..., 2]
            
            x_g_r, y_g_r, z_g_r = transform_gimbal(x_g, y_g, z_g)
            closeup_gimbal_cone = ax_closeup.plot_surface(x_g_r, y_g_r, z_g_r,
                                                          color='cyan', alpha=0.2, linewidth=0)
        
        # Text overlay
        closeup_text.set_text(
            f"Pitch:  {np.rad2deg(att[1]):5.1f}°\n"
            f"Yaw:    {np.rad2deg(att[2]):5.1f}°\n"
            f"Gimbal: ({np.rad2deg(g[0]):5.1f}°, {np.rad2deg(g[1]):5.1f}°)\n"
            f"Thrust: {t:5.0f} N ({thrust_pct*100:4.1f}%)"
        )
        
        # ===== UPDATE MAIN VIEW =====
        
        # Trail with downsampling
        start_idx = max(0, actual_idx - trail_length)
        trail_data = pos[start_idx:actual_idx:CONFIG.viz.trail_downsample_stride]
        if len(trail_data) > 0:
            main_trail.set_data(trail_data[:, 0], trail_data[:, 1])
            main_trail.set_3d_properties(trail_data[:, 2])
        
        # Current position
        main_rocket_dot.set_data([p[0]], [p[1]])
        main_rocket_dot.set_3d_properties([p[2]])
        
        # Closest point
        main_closest_dot.set_data([pos_closest[actual_idx, 0]], [pos_closest[actual_idx, 1]])
        main_closest_dot.set_3d_properties([pos_closest[actual_idx, 2]])
        
        # Update gimbal vector
        main_gimbal_vec.remove()
        main_gimbal_vec = ax_main.quiver(p[0], p[1], p[2],
                                          gimbal_dir_world[0], gimbal_dir_world[1], gimbal_dir_world[2],
                                          length=5.0, color="cyan", linewidth=2, arrow_length_ratio=0.2)
        
        # OPTIMIZATION: Disable rocket body mesh in main view (expensive per-frame operation)
        # The trail and rocket dot are sufficient to show position; detailed model is in closeup view
        # Uncomment the code below if you need the rocket model in the main view
        """
        # Rocket body
        if main_body is not None:
            try:
                main_body.remove()
            except (ValueError, AttributeError):
                pass
        if main_cone is not None:
            try:
                main_cone.remove()
            except (ValueError, AttributeError):
                pass
        
        body_len_main = 6.0  # Match closeup view body length
        body_rad_main = 0.4
        cone_len_main = 1.0
        total_len_main = body_len_main + cone_len_main
        center_offset_main = total_len_main / 2.0
        
        # Body from -center to -center+body_len (centered at 0)
        z0_m = -center_offset_main
        z1_m = z0_m + body_len_main
        cone_z0_m = z1_m
        cone_z1_m = z1_m + cone_len_main
        
        x_cyl_m, y_cyl_m, z_cyl_m = _make_cylinder(body_rad_main, z0_m, z1_m, n=CONFIG.viz.mesh_quality)
        x_cone_m, y_cone_m, z_cone_m = _make_cone(body_rad_main, cone_z0_m, cone_z1_m, n=CONFIG.viz.mesh_quality)
        
        # Body direction (align with body attitude)
        body_dir = R_body @ np.array([0, 0, 1])
        R_main = _rotation_matrix_from_vectors(np.array([0, 0, 1]), body_dir)
        
        def transform_main(x, y, z):
            pts = np.stack([x, y, z], axis=-1)
            shp = pts.shape
            pts_flat = (pts.reshape(-1, 3) @ R_main.T) + p
            return pts_flat.reshape(shp)[..., 0], pts_flat.reshape(shp)[..., 1], pts_flat.reshape(shp)[..., 2]
        
        x_cyl_w, y_cyl_w, z_cyl_w = transform_main(x_cyl_m, y_cyl_m, z_cyl_m)
        x_cone_w, y_cone_w, z_cone_w = transform_main(x_cone_m, y_cone_m, z_cone_m)
        
        main_body = ax_main.plot_surface(x_cyl_w, y_cyl_w, z_cyl_w,
                                         color='lightsteelblue', alpha=0.9, linewidth=0)
        main_cone = ax_main.plot_surface(x_cone_w, y_cone_w, z_cone_w,
                                         color='silver', alpha=0.9, linewidth=0)
        
        # Thrust stick from nozzle (at z0_m)
        stick_len = 2.5 * thrust_pct
        nozzle_local = np.array([0, 0, z0_m])
        stick_base = p + R_main @ nozzle_local
        stick_tip = stick_base + gimbal_dir_world * (-stick_len)
        main_stick.set_data([stick_base[0], stick_tip[0]], [stick_base[1], stick_tip[1]])
        main_stick.set_3d_properties([stick_base[2], stick_tip[2]])
        main_stick.set_color(thrust_color)
        """
        # ===== UPDATE MPC INTERNALS =====
        
        # Position error plot (rolling window)
        window = CONFIG.viz.cost_window_steps
        start_win = max(0, actual_idx - window)
        end_win = actual_idx + 1
        
        error_line.set_data(time[start_win:end_win], pos_error[start_win:end_win])
        ax_error.set_xlim(time[max(0, actual_idx - window)], time[min(len(time)-1, actual_idx + 50)])
        ax_error.set_ylim(0, max(pos_error[start_win:end_win].max() * 1.1, 1.0))
        
        # Fill under curve
        if error_fill is not None:
            error_fill.remove()
        error_fill = ax_error.fill_between(time[start_win:end_win], 0, pos_error[start_win:end_win],
                                           color='red', alpha=0.2)
        error_marker.set_xdata([t_now, t_now])
        
        # MPC Horizon projection
        mpc_pred = mpc_pred_pos[actual_idx]  # Shape (3, N+1)
        mpc_ref = mpc_ref_pos[actual_idx]    # Shape (3, N+1)
        
        if CONFIG.viz.mpc_horizon_projection == "xy":
            horizon_current_dot.set_data([p[0]], [p[1]])
            horizon_ref_line.set_data(mpc_ref[0, :], mpc_ref[1, :])
            horizon_closest_dot.set_data([pos_closest[actual_idx, 0]], [pos_closest[actual_idx, 1]])
            
            # Path segment ahead
            path_start = max(0, actual_idx - 20)
            path_end = min(len(pos_ref), actual_idx + 100)
            horizon_path_line.set_data(pos_ref[path_start:path_end, 0], pos_ref[path_start:path_end, 1])
            
            # Auto-scale
            all_x = np.concatenate([mpc_ref[0, :], [p[0]], pos_ref[path_start:path_end, 0]])
            all_y = np.concatenate([mpc_ref[1, :], [p[1]], pos_ref[path_start:path_end, 1]])
            margin = 10
            ax_horizon.set_xlim(all_x.min() - margin, all_x.max() + margin)
            ax_horizon.set_ylim(all_y.min() - margin, all_y.max() + margin)
        else:  # xz
            horizon_current_dot.set_data([p[0]], [p[2]])
            horizon_ref_line.set_data(mpc_ref[0, :], mpc_ref[2, :])
            horizon_closest_dot.set_data([pos_closest[actual_idx, 0]], [pos_closest[actual_idx, 2]])
            
            # Path segment ahead
            path_start = max(0, actual_idx - 20)
            path_end = min(len(pos_ref), actual_idx + 100)
            horizon_path_line.set_data(pos_ref[path_start:path_end, 0], pos_ref[path_start:path_end, 2])
            
            # Auto-scale
            all_x = np.concatenate([mpc_ref[0, :], [p[0]], pos_ref[path_start:path_end, 0]])
            all_z = np.concatenate([mpc_ref[2, :], [p[2]], pos_ref[path_start:path_end, 2]])
            margin = 10
            ax_horizon.set_xlim(all_x.min() - margin, all_x.max() + margin)
            ax_horizon.set_ylim(all_z.min() - margin, all_z.max() + margin)
        
        # Cost components plot
        cost_pos_line.set_data(time[start_win:end_win], mpc_cost_pos[start_win:end_win])
        cost_vel_line.set_data(time[start_win:end_win], mpc_cost_vel[start_win:end_win])
        cost_accel_line.set_data(time[start_win:end_win], mpc_cost_accel[start_win:end_win])
        cost_jerk_line.set_data(time[start_win:end_win], mpc_cost_jerk[start_win:end_win])
        cost_total_line.set_data(time[start_win:end_win], mpc_cost_total[start_win:end_win])
        
        ax_cost.set_xlim(time[max(0, actual_idx - window)], time[min(len(time)-1, actual_idx + 50)])
        
        # Auto-scale y-axis for costs
        all_costs = np.concatenate([
            mpc_cost_pos[start_win:end_win],
            mpc_cost_vel[start_win:end_win],
            mpc_cost_accel[start_win:end_win],
            mpc_cost_jerk[start_win:end_win],
            mpc_cost_total[start_win:end_win]
        ])
        if len(all_costs) > 0 and all_costs.max() > 0:
            ax_cost.set_ylim(0, all_costs.max() * 1.1)
        
        cost_marker.set_xdata([t_now, t_now])
        
        return (closeup_body, closeup_cone, closeup_thrust_stick, closeup_gimbal_cone,
                main_trail, main_rocket_dot, main_closest_dot, main_gimbal_vec, main_body, main_cone, main_stick,
                error_line, error_fill, error_marker,
                horizon_current_dot, horizon_ref_line, horizon_closest_dot, horizon_path_line,
                cost_pos_line, cost_vel_line, cost_accel_line, cost_jerk_line, cost_total_line, cost_marker)
    
    # ===== CREATE ANIMATION =====
    anim = FuncAnimation(fig, update, frames=num_frames, interval=30, blit=False, repeat=False)
    
    # Suppress tight_layout warning for incompatible axes (Button widgets)
    import warnings
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="This figure includes Axes that are not compatible with tight_layout")
        plt.tight_layout()
    
    plt.show()
    return anim
