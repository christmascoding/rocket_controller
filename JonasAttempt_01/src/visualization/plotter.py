import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


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


def _make_cylinder(radius: float, z0: float, z1: float, n: int = 12):
    theta = np.linspace(0, 2 * np.pi, n)
    z = np.linspace(z0, z1, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x = radius * np.cos(theta_grid)
    y = radius * np.sin(theta_grid)
    return x, y, z_grid


def _make_cone(radius: float, z0: float, z1: float, n: int = 12):
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


def _set_equal_axes(ax, ref_pos: np.ndarray, margin_ratio: float = 0.2):
    finite = np.isfinite(ref_pos).all(axis=1)
    ref = ref_pos[finite]
    if ref.size == 0:
        return
    min_vals = ref.min(axis=0)
    max_vals = ref.max(axis=0)
    center = (min_vals + max_vals) / 2.0
    half_range = (max_vals - min_vals).max() / 2.0
    margin = max(1.0, half_range * margin_ratio)
    max_range = half_range + margin
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)
    ax.set_box_aspect((1.0, 1.0, 1.0))


def animate_trajectory(history: dict, trail_length: int = 500):
    pos = history["position"]
    pos_ref = history["pos_ref"]
    pos_closest = history["pos_closest"]
    gimbal = history["gimbal_angles"]
    thrust = history["thrust_n"]
    velocity = history["velocity"]
    time = history["time"]

    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1.0], height_ratios=[1.0, 1.0, 1.0])
    ax = fig.add_subplot(gs[:, 0], projection="3d")
    ax_thrust = fig.add_subplot(gs[0, 1])
    ax_gimbal = fig.add_subplot(gs[1, 1])
    ax_error = fig.add_subplot(gs[2, 1])

    ax.plot(pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2], "--", color="gray", label="Target")

    actual_line, = ax.plot([], [], [], color="tab:blue", label="Actual")
    point, = ax.plot([], [], [], "o", color="tab:red", markersize=6)
    ref_point, = ax.plot([], [], [], "o", color="tab:purple", markersize=6, label="Closest")
    gimbal_vec = ax.quiver([], [], [], [], [], [], length=5.0, color="tab:green")
    body_surface = None
    cone_surface = None
    stick_line, = ax.plot([], [], [], color="tab:orange", linewidth=3)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.set_title("Rocket Trajectory and Gimbal Direction")

    # Thrust plot
    ax_thrust.plot(time, thrust, color="tab:orange")
    thrust_marker = ax_thrust.axvline(time[0], color="black", linestyle="--")
    ax_thrust.set_title("Thrust (N)")
    ax_thrust.set_xlabel("Time (s)")
    ax_thrust.set_ylabel("Thrust")
    ax_thrust.grid(True, alpha=0.3)

    # Gimbal plot
    ax_gimbal.plot(time, np.rad2deg(gimbal[:, 0]), label="Pitch", color="tab:blue")
    ax_gimbal.plot(time, np.rad2deg(gimbal[:, 1]), label="Yaw", color="tab:green")
    gimbal_marker = ax_gimbal.axvline(time[0], color="black", linestyle="--")
    ax_gimbal.set_title("Gimbal Angles (deg)")
    ax_gimbal.set_xlabel("Time (s)")
    ax_gimbal.set_ylabel("Angle")
    ax_gimbal.grid(True, alpha=0.3)
    ax_gimbal.legend()

    # Position error plot (closest point on path)
    pos_error = np.linalg.norm(pos - pos_closest, axis=1)
    ax_error.plot(time, pos_error, color="tab:red")
    error_marker = ax_error.axvline(time[0], color="black", linestyle="--")
    ax_error.set_title("Position Deviation (m)")
    ax_error.set_xlabel("Time (s)")
    ax_error.set_ylabel("Error")
    ax_error.grid(True, alpha=0.3)

    _set_equal_axes(ax, pos_ref, margin_ratio=0.2)

    speed_factor = 1

    # Speed control buttons
    ax_btn_1x = fig.add_axes([0.78, 0.92, 0.035, 0.05])
    ax_btn_2x = fig.add_axes([0.82, 0.92, 0.035, 0.05])
    ax_btn_4x = fig.add_axes([0.86, 0.92, 0.035, 0.05])
    ax_btn_8x = fig.add_axes([0.90, 0.92, 0.035, 0.05])
    ax_btn_16x = fig.add_axes([0.94, 0.92, 0.035, 0.05])
    ax_btn_32x = fig.add_axes([0.98, 0.92, 0.035, 0.05])
    btn_1x = Button(ax_btn_1x, "1x")
    btn_2x = Button(ax_btn_2x, "2x")
    btn_4x = Button(ax_btn_4x, "4x")
    btn_8x = Button(ax_btn_8x, "8x")
    btn_16x = Button(ax_btn_16x, "16x")
    btn_32x = Button(ax_btn_32x, "32x")

    def _set_speed(val: int):
        nonlocal speed_factor
        speed_factor = val

    btn_1x.on_clicked(lambda _event: _set_speed(1))
    btn_2x.on_clicked(lambda _event: _set_speed(2))
    btn_4x.on_clicked(lambda _event: _set_speed(4))
    btn_8x.on_clicked(lambda _event: _set_speed(8))
    btn_16x.on_clicked(lambda _event: _set_speed(16))
    btn_32x.on_clicked(lambda _event: _set_speed(32))

    def update(frame: int):
        frame = min(frame * speed_factor, len(pos) - 1)
        start = max(0, frame - trail_length)
        segment = pos[start:frame]
        if len(segment) > 0:
            actual_line.set_data(segment[:, 0], segment[:, 1])
            actual_line.set_3d_properties(segment[:, 2])

        point.set_data([pos[frame, 0]], [pos[frame, 1]])
        point.set_3d_properties([pos[frame, 2]])

        ref_point.set_data([pos_closest[frame, 0]], [pos_closest[frame, 1]])
        ref_point.set_3d_properties([pos_closest[frame, 2]])

        # Update gimbal vector (thrust direction)
        pitch, yaw = gimbal[frame]
        body_pitch = history["attitude"][frame, 1]
        body_yaw = history["attitude"][frame, 2]
        thrust_dir = _direction_from_pitch_yaw(pitch, yaw)
        # Rotate gimbal direction by body attitude
        cp = np.cos(body_pitch)
        sp = np.sin(body_pitch)
        cy = np.cos(body_yaw)
        sy = np.sin(body_yaw)
        R_body = np.array(
            [
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ]
        )
        thrust_dir = R_body @ thrust_dir

        nonlocal gimbal_vec
        gimbal_vec.remove()
        gimbal_vec = ax.quiver(
            pos[frame, 0],
            pos[frame, 1],
            pos[frame, 2],
            thrust_dir[0],
            thrust_dir[1],
            thrust_dir[2],
            length=5.0,
            color="tab:green",
        )
        # Draw rocket body (cylinder + cone) aligned with velocity direction
        nonlocal body_surface, cone_surface
        if body_surface is not None:
            body_surface.remove()
        if cone_surface is not None:
            cone_surface.remove()

        body_length = 4.0
        body_radius = 0.4
        cone_length = 1.0
        stick_max_len = 2.5

        center = pos[frame]
        z0 = -body_length / 2.0
        z1 = body_length / 2.0
        cone_z0 = z1
        cone_z1 = z1 + cone_length

        x_cyl, y_cyl, z_cyl = _make_cylinder(body_radius, z0, z1, n=16)
        x_cone, y_cone, z_cone = _make_cone(body_radius, cone_z0, cone_z1, n=16)

        body_dir = R_body @ np.array([0.0, 0.0, 1.0])
        if np.linalg.norm(body_dir) < 1e-6:
            body_dir = np.array([0.0, 0.0, 1.0])
        # Rotate local +Z to body direction
        R = _rotation_matrix_from_vectors(np.array([0.0, 0.0, 1.0]), body_dir)

        def transform(x, y, z):
            pts = np.stack([x, y, z], axis=-1)
            shp = pts.shape
            pts = pts.reshape(-1, 3) @ R.T
            pts = pts.reshape(shp)
            return pts[..., 0] + center[0], pts[..., 1] + center[1], pts[..., 2] + center[2]

        x_cyl_w, y_cyl_w, z_cyl_w = transform(x_cyl, y_cyl, z_cyl)
        x_cone_w, y_cone_w, z_cone_w = transform(x_cone, y_cone, z_cone)

        body_surface = ax.plot_surface(
            x_cyl_w,
            y_cyl_w,
            z_cyl_w,
            color="lightsteelblue",
            alpha=0.9,
            linewidth=0,
        )
        cone_surface = ax.plot_surface(
            x_cone_w,
            y_cone_w,
            z_cone_w,
            color="silver",
            alpha=0.9,
            linewidth=0,
        )

        # Engine stick showing thrust magnitude (aligned with thrust direction)
        stick_len = stick_max_len * (thrust[frame] / max(thrust.max(), 1e-9))
        stick_base = center + R @ np.array([0.0, 0.0, z0])
        stick_tip = stick_base + thrust_dir * (-stick_len)
        stick_line.set_data([stick_base[0], stick_tip[0]], [stick_base[1], stick_tip[1]])
        stick_line.set_3d_properties([stick_base[2], stick_tip[2]])

        thrust_marker.set_xdata([time[frame], time[frame]])
        gimbal_marker.set_xdata([time[frame], time[frame]])
        error_marker.set_xdata([time[frame], time[frame]])

        return (
            actual_line,
            point,
            ref_point,
            gimbal_vec,
            body_surface,
            cone_surface,
            stick_line,
            thrust_marker,
            gimbal_marker,
            error_marker,
        )

    anim = FuncAnimation(fig, update, frames=len(pos), interval=30, blit=False)
    plt.show()
    return anim
