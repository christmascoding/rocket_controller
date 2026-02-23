import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
from src.config import CONFIG


def _rotation_matrix_from_pitch_yaw(pitch: float, yaw: float) -> np.ndarray:
    """Create rotation matrix from pitch and yaw angles (same as simulator)"""
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    return np.array([
        [cy * cp, -sy, cy * sp],
        [sy * cp, cy, sy * sp],
        [-sp, 0.0, cp],
    ], dtype=float)


def _rotation_matrix_from_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Compute rotation matrix from vector a to vector b"""
    a = a / max(np.linalg.norm(a), 1e-9)
    b = b / max(np.linalg.norm(b), 1e-9)
    v = np.cross(a, b)
    c = np.dot(a, b)
    if c < -0.999999:
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / max(np.linalg.norm(v), 1e-9)
        return -np.eye(3) + 2.0 * np.outer(v, v)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    v = v / s
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx * s + (vx @ vx) * (1.0 - c)


def _make_cylinder(radius: float, z0: float, z1: float, n: int = 8):
    """Create cylinder mesh"""
    theta = np.linspace(0, 2 * np.pi, n)
    z = np.linspace(z0, z1, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x = radius * np.cos(theta_grid)
    y = radius * np.sin(theta_grid)
    return x, y, z_grid


def _make_cone(radius: float, z0: float, z1: float, n: int = 8):
    """Create cone mesh"""
    theta = np.linspace(0, 2 * np.pi, n)
    z = np.linspace(z0, z1, 2)
    r = np.linspace(radius, 0.0, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    r_grid, _ = np.meshgrid(r, theta)
    r_grid = r_grid.T
    x = r_grid * np.cos(theta_grid)
    y = r_grid * np.sin(theta_grid)
    return x, y, z_grid


def animate_trajectory(history: dict, trail_length: int = 500):
    """
    Simple, robust 3-panel trajectory visualization:
    - Left: Rocket close-up view (attitude/thrust)
    - Center: Main 3D trajectory (simple lines, no meshes)
    - Right: MPC diagnostics
    """
    # Extract history and convert to numpy arrays
    pos = np.array(history["position"])
    pos_ref = np.array(history["pos_ref"])
    pos_closest = np.array(history["pos_closest"])
    gimbal = np.array(history["gimbal_angles"])
    thrust = np.array(history["thrust_n"])
    attitude = np.array(history["attitude"])
    velocity = np.array(history["velocity"])
    time = np.array(history["time"])
    
    # MPC - handle both list and array formats
    mpc_pred_pos = history.get("mpc_pred_pos", [])
    mpc_ref_pos = history.get("mpc_ref_pos", [])
    mpc_cost_pos = np.array(history.get("mpc_cost_pos", [0.0] * len(time)))
    mpc_cost_vel = np.array(history.get("mpc_cost_vel", [0.0] * len(time)))
    mpc_cost_accel = np.array(history.get("mpc_cost_accel", [0.0] * len(time)))
    mpc_cost_jerk = np.array(history.get("mpc_cost_jerk", [0.0] * len(time)))
    
    max_thrust = thrust.max() if thrust.max() > 1e-6 else 1.0
    
    # Performance settings
    update_interval = CONFIG.viz.update_interval
    num_frames = max(1, len(pos) // update_interval)
    
    # Create figure
    fig = plt.figure(figsize=(22, 10))
    gs = fig.add_gridspec(2, 4, width_ratios=[1.6, 1.2, 0.9, 0.9], 
                          height_ratios=[1.0, 1.0], hspace=0.3, wspace=0.5)
    
    # ===== LEFT: CLOSEUP VIEW =====
    ax_closeup = fig.add_subplot(gs[:, 0], projection="3d")
    ax_closeup.set_title("Rocket Attitude", fontsize=12, fontweight='bold')
    ax_closeup.set_xlabel("X [m]")
    ax_closeup.set_ylabel("Y [m]")
    ax_closeup.set_zlabel("Z [m]")
    ax_closeup.set_xlim(-10, 10)
    ax_closeup.set_ylim(-10, 10)
    ax_closeup.set_zlim(0, 15)
    ax_closeup.view_init(elev=30, azim=45)
    ax_closeup.set_box_aspect((1, 1, 1.5))
    ax_closeup.disable_mouse_rotation()
    ax_closeup.format_coord = lambda x, y: ""
    ax_closeup.grid(True, alpha=0.3)
    
    # Coordinate frame
    ax_closeup.quiver(0, 0, 0, 2, 0, 0, color='r', linewidth=1.5, arrow_length_ratio=0.2)
    ax_closeup.quiver(0, 0, 0, 0, 2, 0, color='g', linewidth=1.5, arrow_length_ratio=0.2)
    ax_closeup.quiver(0, 0, 0, 0, 0, 2, color='b', linewidth=1.5, arrow_length_ratio=0.2)
    
    closeup_body = None
    closeup_cone = None
    closeup_thrust, = ax_closeup.plot([], [], [], linewidth=5, color='orange')
    closeup_text = ax_closeup.text2D(0.02, 0.98, "", transform=ax_closeup.transAxes,
                                      fontsize=9, verticalalignment='top', family='monospace',
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # ===== CENTER: MAIN TRAJECTORY =====
    ax_main = fig.add_subplot(gs[:, 1], projection="3d")
    ax_main.set_title("Rocket Trajectory", fontsize=12, fontweight='bold')
    ax_main.set_xlabel("X [m]")
    ax_main.set_ylabel("Y [m]")
    ax_main.set_zlabel("Z [m]")
    ax_main.view_init(elev=30, azim=45)
    ax_main.format_coord = lambda x, y: ""
    ax_main.grid(True, alpha=0.3)
    
    # Plot reference path ONCE (static)
    ax_main.plot(pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2], 'g--', 
                 alpha=0.6, linewidth=2, label='Target Path')
    
    # Dynamic elements (updated each frame)
    trail_line, = ax_main.plot([], [], [], 'orange', linewidth=2.5, label='Actual Path')
    pos_dot, = ax_main.plot([], [], [], 'ro', markersize=8, label='Current Pos')
    target_dot, = ax_main.plot([], [], [], 'bo', markersize=6, label='MPC Target (k=0)')
    closest_dot, = ax_main.plot([], [], [], 'mo', markersize=5, label='Closest Point')
    
    ax_main.legend(loc='upper right', fontsize=9)
    
    # Set axis limits to show entire trajectory (both ref and actual)
    all_pos = np.vstack([pos_ref, pos])
    mins = all_pos.min(axis=0)
    maxs = all_pos.max(axis=0)
    ranges = maxs - mins
    margins = ranges * 0.2 + 1.0
    ax_main.set_xlim(mins[0] - margins[0], maxs[0] + margins[0])
    ax_main.set_ylim(mins[1] - margins[1], maxs[1] + margins[1])
    ax_main.set_zlim(mins[2] - margins[2], maxs[2] + margins[2])
    
    # ===== RIGHT: MPC INTERNALS =====
    
    # Top-left: Position Error
    ax_error = fig.add_subplot(gs[0, 2])
    ax_error.set_title("Position Error", fontsize=10, fontweight='bold')
    ax_error.set_xlabel("Time [s]")
    ax_error.set_ylabel("Error [m]")
    ax_error.grid(True, alpha=0.3)
    # Ensure pos_error matches position array length
    pos_error = np.linalg.norm(pos[:len(pos_closest)] - pos_closest[:len(pos)], axis=1)
    error_line, = ax_error.plot([], [], 'r-', linewidth=1.5)
    error_marker = ax_error.axvline(0, color='black', linestyle='--', linewidth=1, alpha=0.5)
    
    # Top-right: MPC Horizon
    ax_horizon = fig.add_subplot(gs[0, 3])
    ax_horizon.set_title("MPC Horizon (XY)", fontsize=10, fontweight='bold')
    ax_horizon.set_xlabel("X [m]")
    ax_horizon.set_ylabel("Y [m]")
    ax_horizon.grid(True, alpha=0.3)
    ax_horizon.set_aspect('equal')
    horizon_line, = ax_horizon.plot([], [], 'b-', linewidth=1, alpha=0.7)
    horizon_curr, = ax_horizon.plot([], [], 'ro', markersize=6)
    
    # Bottom-left: Costs
    ax_costs = fig.add_subplot(gs[1, 2])
    ax_costs.set_title("MPC Costs", fontsize=10, fontweight='bold')
    ax_costs.set_xlabel("Time [s]")
    ax_costs.set_ylabel("Cost")
    ax_costs.grid(True, alpha=0.3)
    cost_pos, = ax_costs.plot([], [], label='Pos', linewidth=1)
    cost_vel, = ax_costs.plot([], [], label='Vel', linewidth=1)
    cost_accel, = ax_costs.plot([], [], label='Accel', linewidth=1)
    ax_costs.legend(fontsize=8, loc='upper right')
    
    # Bottom-right: Reserved
    ax_info = fig.add_subplot(gs[1, 3])
    ax_info.axis('off')
    info_text = ax_info.text(0.05, 0.95, "", transform=ax_info.transAxes,
                            fontsize=9, verticalalignment='top', family='monospace',
                            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    # ===== TIMELINE SEEKBAR =====
    ax_timeline = fig.add_axes([0.1, 0.975, 0.55, 0.015])
    slider = Slider(ax_timeline, 'Frame', 0, num_frames - 1, valinit=0, valstep=1, color='steelblue')
    
    # ===== PLAYBACK CONTROLS =====
    play_state = {'is_playing': True, 'current_frame': 0, 'user_seeking': False, 'last_slider_val': 0}
    
    def on_slider_change(val):
        """User moved slider - only trigger if value actually changed"""
        if int(val) != play_state['last_slider_val']:
            play_state['user_seeking'] = True
            play_state['current_frame'] = int(val)
            play_state['last_slider_val'] = int(val)
            fig.canvas.draw_idle()
    
    slider.on_changed(on_slider_change)
    
    # Play/Pause button
    ax_play = fig.add_axes([0.65, 0.96, 0.04, 0.03])
    btn_play = Button(ax_play, 'Play/Pause', color='lightblue', hovercolor='skyblue')
    
    def on_play_click(_event):
        """Toggle playback"""
        play_state['is_playing'] = not play_state['is_playing']
        if play_state['is_playing']:
            anim.resume()
        else:
            anim.pause()
    
    btn_play.on_clicked(on_play_click)
    
    # Speed buttons
    speeds = [1, 2, 4, 8, 16, 32]
    for i, speed in enumerate(speeds):
        ax_btn = fig.add_axes([0.695 + i * 0.035, 0.96, 0.03, 0.03])
        btn = Button(ax_btn, f'{speed}x', color='lightgray', hovercolor='gray')
        
        def make_speed_callback(s):
            def callback(_event):
                anim.event_source.interval = max(1, int(30 / s))
            return callback
        
        btn.on_clicked(make_speed_callback(speed))
    
    # ===== VIEW SYNCHRONIZATION =====
    view_sync = {'last_elev': 30, 'last_azim': 45}
    
    def on_motion(event):
        """Sync closeup view when user rotates main trajectory view"""
        if event.inaxes != ax_main:
            return
        try:
            # Try to read current view angles from ax_main
            elev = getattr(ax_main, 'elev', None)
            azim = getattr(ax_main, 'azim', None)
            if elev is not None and azim is not None:
                # If angles changed, apply to closeup
                if abs(elev - view_sync['last_elev']) > 0.1 or abs(azim - view_sync['last_azim']) > 0.1:
                    ax_closeup.view_init(elev=elev, azim=azim)
                    view_sync['last_elev'] = elev
                    view_sync['last_azim'] = azim
                    fig.canvas.draw_idle()
        except:
            pass
    
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    
    # ===== ANIMATION UPDATE FUNCTION =====
    def update(frame_idx: int):
        nonlocal closeup_body, closeup_cone
        
        # If user is seeking via slider, use their choice and pause
        if play_state['user_seeking']:
            current_frame = play_state['current_frame']
            play_state['is_playing'] = False
            play_state['user_seeking'] = False
        # Otherwise, advance frame if playing
        elif play_state['is_playing']:
            play_state['current_frame'] = min(frame_idx, num_frames - 1)
            current_frame = play_state['current_frame']
        else:
            # Paused: stay at current frame
            current_frame = play_state['current_frame']
        
        actual_idx = min(current_frame * update_interval, len(pos) - 1)
        
        # Current state
        p = pos[actual_idx]
        g = gimbal[actual_idx]
        t = thrust[actual_idx]
        att = attitude[actual_idx]
        t_now = time[actual_idx]
        
        # Body attitude from actual simulation state (not velocity-aligned)
        R_body = _rotation_matrix_from_pitch_yaw(att[1], att[2])
        
        # Gimbal direction using same formula as simulator
        # direction_from_angles returns [cos(yaw)*sin(pitch), sin(yaw)*sin(pitch), cos(pitch)]
        # But gimbal points BACKWARD (negative Z), so negate Z component
        # Also negate yaw since gimbal command has inverted yaw for torque control
        pitch, yaw = g[0], -g[1]
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        gimbal_dir_body = np.array([cy * sp, sy * sp, -cp])  # Negative cp for backward direction
        gimbal_dir_world = R_body @ gimbal_dir_body
        
        # ===== UPDATE CLOSEUP VIEW =====
        
        # Remove old meshes
        if closeup_body is not None:
            try:
                closeup_body.remove()
            except:
                pass
        if closeup_cone is not None:
            try:
                closeup_cone.remove()
            except:
                pass
        
        # Create rocket meshes (low quality for performance)
        body_rad = 0.8
        cone_rad = 0.8
        body_len = 6.0
        cone_len = 1.0
        center_offset = 3.5
        
        z0, z1 = -center_offset, center_offset - 1.0
        zc0, zc1 = z1, z1 + cone_len
        
        x_cyl, y_cyl, z_cyl = _make_cylinder(body_rad, z0, z1, n=6)
        x_cone, y_cone, z_cone = _make_cone(cone_rad, zc0, zc1, n=6)
        
        # Rotate and plot
        def rotate_mesh(x, y, z, R):
            pts = np.stack([x, y, z], axis=-1)
            shp = pts.shape
            pts_rot = (pts.reshape(-1, 3) @ R.T).reshape(shp)
            return pts_rot[..., 0], pts_rot[..., 1], pts_rot[..., 2]
        
        x_cyl_r, y_cyl_r, z_cyl_r = rotate_mesh(x_cyl, y_cyl, z_cyl, R_body)
        x_cone_r, y_cone_r, z_cone_r = rotate_mesh(x_cone, y_cone, z_cone, R_body)
        
        closeup_body = ax_closeup.plot_surface(x_cyl_r, y_cyl_r, z_cyl_r,
                                               color='lightsteelblue', alpha=0.9, linewidth=0)
        closeup_cone = ax_closeup.plot_surface(x_cone_r, y_cone_r, z_cone_r,
                                              color='silver', alpha=0.9, linewidth=0)
        
        # Thrust vector
        thrust_pct = t / max_thrust
        thrust_color = '#FF4500' if thrust_pct > 0.8 else '#FFA500' if thrust_pct > 0.4 else '#FFD700'
        nozzle_pos = np.array([0, 0, -center_offset])
        nozzle_world = R_body @ nozzle_pos
        thrust_tip = nozzle_world + gimbal_dir_world * 8.0 * thrust_pct
        closeup_thrust.set_data([nozzle_world[0], thrust_tip[0]], 
                               [nozzle_world[1], thrust_tip[1]])
        closeup_thrust.set_3d_properties([nozzle_world[2], thrust_tip[2]])
        closeup_thrust.set_color(thrust_color)
        
        info_str = f"Pos: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})\nAtt: ({att[0]:.2f}, {att[1]:.2f})\nThrust: {thrust_pct:.1%}"
        closeup_text.set_text(info_str)
        
        # ===== UPDATE MAIN VIEW =====
        
        # Trail (actual trajectory so far)
        start_idx = max(0, actual_idx - trail_length)
        trail_data = pos[start_idx:actual_idx+1]
        if len(trail_data) > 0:
            trail_line.set_data(trail_data[:, 0], trail_data[:, 1])
            trail_line.set_3d_properties(trail_data[:, 2])
        
        # Current position
        pos_dot.set_data([p[0]], [p[1]])
        pos_dot.set_3d_properties([p[2]])

        # MPC target (k=0 in reference horizon)
        try:
            if isinstance(mpc_ref_pos, list) and len(mpc_ref_pos) > actual_idx:
                ref_seq = mpc_ref_pos[actual_idx]
                if hasattr(ref_seq, '__len__') and len(ref_seq) > 0:
                    target = ref_seq[0] if hasattr(ref_seq[0], '__len__') else ref_seq
                    target_dot.set_data([target[0]], [target[1]])
                    target_dot.set_3d_properties([target[2]])
            elif hasattr(mpc_ref_pos, 'shape') and mpc_ref_pos.shape[0] > actual_idx:
                ref_seq = mpc_ref_pos[actual_idx]
                if ref_seq.size > 0:
                    if ref_seq.shape[0] == 3:
                        target = ref_seq[:, 0]
                    else:
                        target = ref_seq[0]
                    target_dot.set_data([target[0]], [target[1]])
                    target_dot.set_3d_properties([target[2]])
        except (IndexError, ValueError, TypeError):
            pass

        # Closest point on path (projection)
        try:
            if len(pos_closest) > actual_idx:
                pc = pos_closest[actual_idx]
                closest_dot.set_data([pc[0]], [pc[1]])
                closest_dot.set_3d_properties([pc[2]])
        except (IndexError, ValueError, TypeError):
            pass
        
        # ===== UPDATE RIGHT PANEL =====
        
        # Error plot
        window = 200
        start_w = max(0, actual_idx - window)
        end_w = actual_idx + 1
        error_line.set_data(time[start_w:end_w], pos_error[start_w:end_w])
        ax_error.set_xlim(time[max(0, actual_idx - window)], time[min(len(time)-1, actual_idx + 100)])
        ax_error.set_ylim(0, max(pos_error[start_w:end_w].max() * 1.1, 0.5))
        error_marker.set_xdata([t_now, t_now])
        
        # MPC Horizon
        try:
            if isinstance(mpc_pred_pos, list) and len(mpc_pred_pos) > actual_idx:
                pred = mpc_pred_pos[actual_idx]
                if hasattr(pred, '__len__') and len(pred) > 0:
                    horizon_line.set_data(pred[:, 0], pred[:, 1])
                    horizon_curr.set_data([p[0]], [p[1]])
                    ax_horizon.relim()
                    ax_horizon.autoscale_view()
            elif hasattr(mpc_pred_pos, 'shape') and mpc_pred_pos.shape[0] > actual_idx:
                pred = mpc_pred_pos[actual_idx]
                if len(pred) > 0:
                    horizon_line.set_data(pred[:, 0], pred[:, 1])
                    horizon_curr.set_data([p[0]], [p[1]])
                    ax_horizon.relim()
                    ax_horizon.autoscale_view()
        except (IndexError, ValueError, TypeError):
            pass
        
        # Costs
        start_w = max(0, actual_idx - 300)
        cost_pos.set_data(time[start_w:end_w], mpc_cost_pos[start_w:end_w])
        cost_vel.set_data(time[start_w:end_w], mpc_cost_vel[start_w:end_w])
        cost_accel.set_data(time[start_w:end_w], mpc_cost_accel[start_w:end_w])
        ax_costs.relim()
        ax_costs.autoscale_view()
        
        info_text.set_text(f"Frame: {current_frame}/{num_frames}\nTime: {t_now:.2f}s")
        
        # Update slider position ONLY during automatic playback (not during user seeking)
        # This prevents the slider from jumping back when user tries to seek
        if not play_state['user_seeking'] and play_state['is_playing']:
            play_state['last_slider_val'] = current_frame
            slider.set_val(current_frame)
        
        return (trail_line, pos_dot, target_dot, closest_dot, closeup_thrust, closeup_body, closeup_cone,
                error_line, error_marker, horizon_line, horizon_curr,
                cost_pos, cost_vel, cost_accel, closeup_text, info_text)
    
    # Create animation
    anim = FuncAnimation(fig, update, frames=num_frames, interval=30, blit=False, repeat=False)
    
    import warnings
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="This figure includes Axes that are not compatible with tight_layout")
        plt.tight_layout()
    
    plt.show()
