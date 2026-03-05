import numpy as np
from scipy.linalg import solve_continuous_are

from src import config
from src.utils import clamp, body_z_axis_in_world, euler_to_dcm


def normalize_angle(angle):
    """Wrap angle to [-pi, pi] range."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def pure_pursuit(rocket_pos, traj_pos):
    """
    Pure pursuit guidance: find carrot point on trajectory and return desired pitch angle.
    Returns: (theta_des, carrot_pos, end_of_path_flag)
    """
    # Find closest point on trajectory
    dists = np.linalg.norm(traj_pos[:, [0, 2]] - rocket_pos[[0, 2]], axis=1)
    closest_idx = np.argmin(dists)
    
    # Check if near end of path
    end_threshold = int(len(traj_pos) * config.PURE_PURSUIT_END_THRESHOLD)
    if closest_idx >= end_threshold:
        # Hold current attitude, disable pure pursuit
        return 0.0, traj_pos[-1], True
    
    # Lookahead: find point at lookahead distance ahead on path
    cumulative_dist = 0.0
    target_idx = closest_idx
    for i in range(closest_idx + 1, len(traj_pos)):
        seg_dist = np.linalg.norm(traj_pos[i, [0, 2]] - traj_pos[i - 1, [0, 2]])
        cumulative_dist += seg_dist
        if cumulative_dist >= config.PURE_PURSUIT_LOOKAHEAD:
            target_idx = i
            break
    else:
        target_idx = len(traj_pos) - 1
    
    # Carrot point
    x_carrot = traj_pos[target_idx, 0]
    z_carrot = traj_pos[target_idx, 2]
    
    # Vector from rocket to carrot
    dx = x_carrot - rocket_pos[0]
    dz = z_carrot - rocket_pos[2]
    
    # Desired pitch angle (0 rad = straight up along Z)
    theta_des = np.arctan2(dx, dz)
    theta_des = clamp(theta_des, -config.MAX_PITCH_FROM_VERTICAL, config.MAX_PITCH_FROM_VERTICAL)
    
    return theta_des, np.array([x_carrot, 0.0, z_carrot]), False


def outer_loop(pos, vel, pos_des, vel_des, mass):
    acc_cmd = config.Kp_pos * (pos_des - pos) + config.Kd_pos * (vel_des - vel)
    f_des = mass * (np.array([0.0, 0.0, config.G]) + acc_cmd)
    return f_des


def middle_loop(f_des, max_thrust, tilt_gain=1.0):
    f_adj = f_des.copy()
    f_adj[0:2] *= tilt_gain
    mag = np.linalg.norm(f_adj)
    if mag < 1e-6:
        return np.array([0.0, 0.0, 1.0]), 0.0
    u_des = f_adj / mag
    throttle = clamp(mag / max_thrust, 0.0, 1.0)
    return u_des, throttle


def inner_loop(
    phi,
    theta,
    psi,
    omega,
    u_des,
    throttle,
    max_thrust,
    cg_from_gimbal,
    gimbal_limit=None,
    kp=None,
    kd=None,
):
    """Inner loop attitude controller.
    
    Args:
        kp, kd: Optional override gains. If None, uses config.Kp_att and config.Kd_att
    """
    if kp is None:
        kp = config.Kp_att
    if kd is None:
        kd = config.Kd_att
    
    u_body_world = body_z_axis_in_world(phi, theta, psi)
    error_vec = np.cross(u_body_world, u_des)

    # PD torque command in body coordinates (approx by projecting error into body frame)
    torque_world = kp * error_vec - kd * omega
    dcm = euler_to_dcm(phi, theta, psi)
    torque_body = dcm.T @ torque_world

    # Convert desired torque into gimbal commands (small-angle model)
    thrust = max_thrust * throttle
    if thrust < 1e-3:
        return 0.0, 0.0

    delta_y = torque_body[1] / (cg_from_gimbal * thrust)
    delta_z = -torque_body[0] / (cg_from_gimbal * thrust)

    limit = config.GIMBAL_MAX if gimbal_limit is None else gimbal_limit
    delta_y = clamp(delta_y, -limit, limit)
    delta_z = clamp(delta_z, -limit, limit)

    return delta_y, delta_z


def aero_surface_loop(phi, theta, psi, omega, u_des, kp, kd, invert=False):
    u_body_world = body_z_axis_in_world(phi, theta, psi)
    error_vec = np.cross(u_body_world, u_des)
    if invert:
        error_vec = -error_vec
        omega = -omega
    torque_world = kp * error_vec - kd * omega
    dcm = euler_to_dcm(phi, theta, psi)
    torque_body = dcm.T @ torque_world

    # Limit aerodynamic torque authority
    torque_body = clamp(torque_body, -config.AERO_TORQUE_MAX, config.AERO_TORQUE_MAX)
    return torque_body


def lqr_gain(mass, inertia, thrust, cg_from_gimbal):
    """LQR gain for vertical hoverslam using BOTH gimbal axes.

    State: [vx, vy, theta, phi, q, p]
      - vx, vy: lateral velocities (world frame)
      - theta: pitch angle (tilt toward +x)
      - phi: roll angle (tilt toward +y)
      - q, p: pitch/roll rates

    Control: [delta_y, delta_z]
      - delta_y: gimbal pitch → My torque → controls theta
      - delta_z: gimbal yaw → Mx torque → controls phi (lateral Y tilt)

    This directly uses BOTH gimbal axes for lateral stabilization.
    """
    g = config.G
    ixx = inertia[0, 0]  # Roll inertia
    iyy = inertia[1, 1]  # Pitch inertia

    # 6x6 A matrix
    a = np.zeros((6, 6))
    # Vertical equilibrium: small tilt creates lateral acceleration
    a[0, 2] = -g    # vx_dot ≈ -g * theta
    a[1, 3] = g     # vy_dot ≈  g * phi
    a[2, 4] = 1.0   # theta_dot = q
    a[3, 5] = 1.0   # phi_dot = p

    # 6x2 B matrix (gimbal only)
    b = np.zeros((6, 2))
    lever_arm = cg_from_gimbal
    if lever_arm < 0.1:
        lever_arm = 0.1

    torque_pitch = thrust * lever_arm
    if torque_pitch < 1.0:
        torque_pitch = 1.0
    torque_roll = torque_pitch

    b[4, 0] = torque_pitch / iyy   # q_dot from delta_y
    b[5, 1] = -torque_roll / ixx   # p_dot from delta_z (negative from cross product)

    # LQR weights
    q_weights = np.diag([8.0, 8.0, 25.0, 25.0, 50.0, 50.0])
    r_weights = np.diag([50.0, 50.0])

    p = solve_continuous_are(a, b, q_weights, r_weights)
    k = np.linalg.inv(r_weights) @ b.T @ p
    return k


def phase2_flip_controller(phi, theta, psi, omega):
    """
    Smooth 180-degree flip controller for Phase 2.
    Target: pitch = pi (nose up, engine down)
    Returns: torque command in body frame
    """
    # Desired attitude: 180 deg pitch (pi radians)
    theta_des = np.pi
    phi_des = 0.0
    
    # Angle errors
    theta_err = theta_des - theta
    phi_err = phi_des - phi
    
    # Wrap theta error to [-pi, pi]
    while theta_err > np.pi:
        theta_err -= 2.0 * np.pi
    while theta_err < -np.pi:
        theta_err += 2.0 * np.pi
    
    # PD controller with strong damping
    torque_body = np.array([
        config.PHASE2_FLIP_Kp * phi_err - config.PHASE2_FLIP_Kd * omega[0],
        config.PHASE2_FLIP_Kp * theta_err - config.PHASE2_FLIP_Kd * omega[1],
        -config.PHASE2_RATE_DAMP * omega[2],  # Pure rate damping for yaw
    ])
    
    # Additional pure rate damping to kill any residual spin
    torque_body -= config.PHASE2_RATE_DAMP * omega
    
    return clamp(torque_body, -config.AERO_TORQUE_MAX, config.AERO_TORQUE_MAX)


def phase1c_controller(state, rocket, time_in_phase1c=0.0):
    """Powered flip controller for apogee maneuver (Phase 1c).
    
    Aligns rocket with retrograde (engine-first) for stable descent entry.
    Uses gimbal as primary control with low throttle for control authority.
    Uses vector-based control to avoid angle wrapping issues.
    """
    pos = state[0:3]
    vel = state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]
    
    # Constant low throttle for gimbal authority
    throttle = config.PHASE1C_THROTTLE
    
    # Get the rocket's current body Z-axis (nose direction) in world frame
    u_rocket = body_z_axis_in_world(phi, theta, psi)  # [ux, uy, uz] - nose direction
    
    # Get desired direction: retrograde (opposite to velocity)
    v_mag = np.linalg.norm(vel)
    if v_mag > 5.0:
        u_desired = -vel / v_mag  # Point opposite to velocity vector
    else:
        # At very low velocity, maintain current attitude
        u_desired = u_rocket
    
    # Compute cross product to get rotation axis (what axis to rotate around)
    rotation_axis = np.cross(u_rocket, u_desired)
    rotation_axis_mag = np.linalg.norm(rotation_axis)
    
    # Compute angle between current and desired direction using dot product
    cos_angle = np.clip(np.dot(u_rocket, u_desired), -1.0, 1.0)
    angle_error = np.arccos(cos_angle)  # This is always in [0, π], no wrapping!
    
    if rotation_axis_mag > 1e-6 and angle_error > 1e-4:
        # Normalize rotation axis
        rotation_axis_norm = rotation_axis / rotation_axis_mag
        
        # Compute desired angular velocity for smooth convergence
        # Proportional to error angle, limited by max angular acceleration
        omega_desired_mag = config.PHASE1C_FLIP_KP * angle_error
        omega_desired = rotation_axis_norm * omega_desired_mag
        
        # Compute angular velocity error
        omega_body = state[9:12]
        omega_error = omega_desired - omega_body
        
        # Use derivative term for damping
        # tau = M * alpha, where alpha is angular acceleration
        # We want: omega_error = -KD * (omega_body - omega_desired)
        # So: tau = M * KD * (omega_desired - omega_body)
        tau_body = config.PHASE1C_FLIP_KD * omega_error
        
        # Convert body-frame torque to gimbal commands
        # tau_pitch uses gimbal_y, tau_yaw uses gimbal_z
        # For small gimbal angles: tau ≈ F * L * gimbal
        F = throttle * rocket.max_thrust
        L = rocket.cg_from_gimbal
        
        gimbal_pitch = tau_body[1] / (F * L + 1e-6)
        gimbal_yaw = tau_body[2] / (F * L + 1e-6)
    else:
        # Already aligned or near singular
        gimbal_pitch = 0.0
        gimbal_yaw = 0.0
        angle_error = 0.0
    
    # Clamp gimbal angles to physical limits
    delta_y = clamp(gimbal_pitch, -config.GIMBAL_MAX, config.GIMBAL_MAX)
    delta_z = clamp(gimbal_yaw, -config.GIMBAL_MAX, config.GIMBAL_MAX)
    
    # Diagnostic output every 0.5s
    if time_in_phase1c % 0.5 < 0.05:
        q_mag = np.sqrt(omega[1]**2 + omega[2]**2)
        print(f"  FLIP t={time_in_phase1c:.2f}s: v_mag={v_mag:.1f}m/s, angle_err={np.rad2deg(angle_error):.1f}°")
        print(f"         u_rocket=[{u_rocket[0]:.3f}, {u_rocket[1]:.3f}, {u_rocket[2]:.3f}]")
        print(f"         u_desired=[{u_desired[0]:.3f}, {u_desired[1]:.3f}, {u_desired[2]:.3f}]")
        print(f"         ω_mag={np.rad2deg(q_mag):.1f}°/s, δy={np.rad2deg(delta_y):.1f}°, δz={np.rad2deg(delta_z):.1f}°")
    
    return {
        "throttle": throttle,
        "delta_y": delta_y,
        "delta_z": delta_z,
        "aero_torque": np.zeros(3),
        "grid_fins": 1.0,  # Deploy grid fins for CP shift
    }


def phase3_controller(state, rocket, time_in_phase3=0.0, prev_gimbal=(0.0, 0.0)):
    pos = state[0:3]
    vel = state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]

    # Energy-optimal vertical throttle control (suicide burn)
    z = pos[2]
    vz = vel[2]
    
    if z > 0.5 and vz < 0.0:
        # Calculate required deceleration to null velocity at ground
        # Energy equation: v² = 2*a*d, so a_req = v²/(2*d)
        # Add safety factor 1.5x to ensure aggressive deceleration
        a_req = (vz ** 2) / (2.0 * z) * 1.5
        thrust_req = rocket.mass * (a_req + config.G)
        throttle = clamp(thrust_req / rocket.max_thrust, config.THROTTLE_MIN_PHASE3, 1.0)
    else:
        # Near ground or ascending - minimal throttle
        throttle = config.THROTTLE_MIN_PHASE3

    # LQR for attitude/velocity stabilization (uses BOTH gimbal axes)
    # CRITICAL: Recompute LQR gain with ACTUAL thrust for correct linearization
    thrust = throttle * rocket.max_thrust
    try:
        # Dynamic LQR gain based on current thrust (not hover thrust)
        k = lqr_gain(rocket.mass, rocket.inertia, thrust, rocket.cg_from_gimbal)
        
        # Normalize angles to avoid wrap-around
        theta_normalized = normalize_angle(theta)
        phi_normalized = normalize_angle(phi)
        
        # Retrograde tilt targets based on horizontal velocity
        # vx_dot ≈ -g*theta => theta_des = (gain * vx/g)
        # vy_dot ≈  g*phi  => phi_des  = (-gain * vy/g)
        tilt_max = config.PHASE3_RETRO_TILT_MAX
        theta_des = clamp(config.PHASE3_RETRO_TILT_GAIN * vel[0] / config.G, -tilt_max, tilt_max)
        phi_des = clamp(-config.PHASE3_RETRO_TILT_GAIN * vel[1] / config.G, -tilt_max, tilt_max)
        
        # Target: retrograde tilt
        theta_err = normalize_angle(theta_normalized - theta_des)
        phi_err = normalize_angle(phi_normalized - phi_des)
        
        # Diagnostic output at ignition
        if time_in_phase3 < 0.1:
            print(f"  PHASE3 INIT: theta_raw={np.rad2deg(theta):.1f}°, theta_norm={np.rad2deg(theta_normalized):.1f}°, "
                f"theta_err={np.rad2deg(theta_err):.1f}°, phi_err={np.rad2deg(phi_err):.1f}°, "
                f"theta_des={np.rad2deg(theta_des):.1f}°, phi_des={np.rad2deg(phi_des):.1f}°, "
                f"vx={vel[0]:.1f}m/s, vy={vel[1]:.1f}m/s")
            print(f"       LQR GAIN K =")
            print(f"         K[0,:] (δy from x) = {k[0,:]}")
            print(f"         K[1,:] (δz from x) = {k[1,:]}")
        
        # Soft engagement: ramp up gains gradually
        if time_in_phase3 < config.PHASE3_RATE_PRIORITY_TIME:
            # First 0.2s: Pure rate damping only (ignore position/velocity)
            x = np.array([0.0, 0.0, 0.0, 0.0, omega[1], omega[0]])
            gain_ramp = 0.0
        elif z > config.PHASE3_ATTITUDE_ONLY_ALT:
            # Above 50m: Attitude-only mode (ignore horizontal drift for max vertical thrust)
            x = np.array([0.0, 0.0, theta_err, phi_err, omega[1], omega[0]])
            gain_ramp = 0.0
        elif time_in_phase3 < config.PHASE3_SOFT_ENGAGEMENT_TIME:
            # 0.2s to 0.5s AND below 50m: Gradually blend in position/velocity errors
            blend = (time_in_phase3 - config.PHASE3_RATE_PRIORITY_TIME) / \
                    (config.PHASE3_SOFT_ENGAGEMENT_TIME - config.PHASE3_RATE_PRIORITY_TIME)
            gain_ramp = blend  # 0 to 1
            x = np.array([
                vel[0] * gain_ramp,
                vel[1] * gain_ramp,
                theta_err * gain_ramp,
                phi_err * gain_ramp,
                omega[1],
                omega[0]
            ])
        else:
            # After 0.5s AND below 50m: Full state feedback
            gain_ramp = 1.0
            x = np.array([vel[0], vel[1], theta_err, phi_err, omega[1], omega[0]])
        
        # Compute raw LQR command
        u = -k @ x
        delta_y_body = u[0]  # gimbal pitch command
        delta_z_body = u[1]  # gimbal yaw command
        
        # DEBUG: Show LQR computation at key moments
        if time_in_phase3 < 2.0 and time_in_phase3 % 0.1 < 0.06:
            print(f"       LQR u = -K @ x: delta_y={u[0]:.3f} rad, delta_z={u[1]:.3f} rad")
        
        # HARD CLAMP gimbal (prevent simulation crashes)
        delta_y = clamp(delta_y_body, -config.GIMBAL_MAX_PHASE3, config.GIMBAL_MAX_PHASE3)
        delta_z = clamp(delta_z_body, -config.GIMBAL_MAX_PHASE3, config.GIMBAL_MAX_PHASE3)
        aero_torque = np.zeros(3)
        
        # Diagnostic output
        if time_in_phase3 < 2.0 and time_in_phase3 % 0.1 < 0.06:
            print(f"       OUTPUT: δy={np.rad2deg(delta_y):.1f}°, δz={np.rad2deg(delta_z):.1f}°")
        
    except Exception as e:
        print(f"LQR failed: {e}, falling back to inner loop")
        u_des = np.array([0.0, 0.0, 1.0])
        delta_y, delta_z = inner_loop(
            phi,
            theta,
            psi,
            omega,
            u_des,
            throttle,
            rocket.max_thrust,
            rocket.cg_from_gimbal,
        )
        aero_torque = np.zeros(3)  # No aero control in fallback

    return throttle, delta_y, delta_z, aero_torque
