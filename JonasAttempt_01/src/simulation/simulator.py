from dataclasses import dataclass
import numpy as np

from src.config import Config, deg2rad
from src.controllers.mpc import MPCController, MPCParams
from src.controllers.lqr import LQRTrajectoryController
from src.models.actuators import ThrustActuator, GimbalActuator
from src.models.trajectory import HelixTrajectory, UpwardCurveTrajectory, ParabolicCruiseTrajectory
from src.debug_monitor import DebugMonitor, attach_debug_info


def direction_from_angles(pitch: float, yaw: float) -> np.ndarray:
    # Roll is fixed at 0 for this simplified model
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    # Direction in world frame for yaw (Z) then pitch (Y)
    return np.array([cy * sp, sy * sp, cp], dtype=float)


def rotation_matrix_from_pitch_yaw(pitch: float, yaw: float) -> np.ndarray:
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    # Rz(yaw) * Ry(pitch)
    return np.array(
        [
            [cy * cp, -sy, cy * sp],
            [sy * cp, cy, sy * sp],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )


def pitch_yaw_from_direction(vec: np.ndarray) -> tuple[float, float]:
    v = vec / max(np.linalg.norm(vec), 1e-9)
    yaw = np.arctan2(v[1], v[0])
    # Rotate into yaw frame to get signed pitch
    cy = np.cos(-yaw)
    sy = np.sin(-yaw)
    x_rot = cy * v[0] - sy * v[1]
    z_rot = v[2]
    pitch = np.arctan2(x_rot, z_rot)
    return pitch, yaw


@dataclass
class RocketState:
    position: np.ndarray
    velocity: np.ndarray
    attitude: np.ndarray  # [roll, pitch, yaw]
    angular_velocity: np.ndarray


class RocketSimulator:
    def __init__(self, config: Config):
        self.config = config
        self.mass = config.physical.mass_kg
        self.gravity = np.array([0.0, 0.0, -config.physical.gravity], dtype=float)

        self.thrust_actuator = ThrustActuator(
            time_constant_s=config.actuators.thrust_time_constant_s,
            rate_limit_n_per_s=config.actuators.thrust_rate_limit_n_per_s,
        )

        self.gimbal_actuator = GimbalActuator(
            max_angle_rad=deg2rad(config.actuators.gimbal_max_deg),
            rate_limit_rad_per_s=deg2rad(config.actuators.gimbal_rate_limit_deg_per_s),
            accel_limit_rad_per_s2=deg2rad(config.actuators.gimbal_accel_limit_deg_per_s2),
        )

        # Gimbal offset from center of mass (engine is behind center)
        self.gimbal_offset = config.physical.gimbal_offset_m

        self.mpc = MPCController(
            MPCParams(
                horizon_steps=config.mpc.horizon_steps,
                dt=config.sim.dt,
                weight_pos=config.mpc.weight_pos,
                weight_vel=config.mpc.weight_vel,
                weight_accel=config.mpc.weight_accel,
                weight_lateral_accel=config.mpc.weight_lateral_accel,
                weight_jerk_thrust=config.mpc.weight_jerk_thrust,
                max_accel=config.mpc.max_accel,
                max_lateral_accel=config.mpc.max_lateral_accel,
            )
        )

        # Initialize LQR trajectory controller
        self.lqr = LQRTrajectoryController(config.lqr)

        if config.sim.trajectory_type == "helix":
            self.trajectory = HelixTrajectory(
                radius_m=config.sim.trajectory_radius_m,
                height_m=config.sim.trajectory_height_m,
                turns=config.sim.trajectory_turns,
                total_time=config.sim.total_time,
            )
        elif config.sim.trajectory_type == "parabolic_cruise":
            self.trajectory = ParabolicCruiseTrajectory(
                peak_m=config.sim.parabola_peak_m,
                ascent_length_m=config.sim.parabola_ascent_length_m,
                cruise_length_m=config.sim.parabola_cruise_length_m,
                descent_length_m=config.sim.parabola_descent_length_m,
                ascent_time_s=config.sim.parabola_ascent_time_s,
                cruise_time_s=config.sim.parabola_cruise_time_s,
                descent_time_s=config.sim.parabola_descent_time_s,
            )
        else:
            self.trajectory = UpwardCurveTrajectory(
                height_m=config.sim.trajectory_height_m,
                curve_amp_m=config.sim.curve_amp_m,
                total_time=config.sim.total_time,
            )

        pos_ref, vel_ref, acc_ref = self.trajectory.sample(0.0)
        self.state = RocketState(
            position=pos_ref.copy(),
            velocity=vel_ref.copy(),
            attitude=np.zeros(3),
            angular_velocity=np.zeros(3),
        )

        # Initialize actuators and attitude to align with initial desired thrust
        desired_thrust_vec = self.mass * (acc_ref - self.gravity)
        desired_thrust_mag = np.linalg.norm(desired_thrust_vec)
        if desired_thrust_mag < 1e-6:
            desired_dir = np.array([0.0, 0.0, 1.0])
        else:
            desired_dir = desired_thrust_vec / desired_thrust_mag

        # Feasibility limit: constrain desired direction to gimbal max tilt
        max_tilt = deg2rad(self.config.actuators.gimbal_max_deg)
        lateral_mag = np.hypot(desired_dir[0], desired_dir[1])
        tilt = np.arctan2(lateral_mag, desired_dir[2])
        if tilt > max_tilt:
            scale = np.tan(max_tilt) / max(lateral_mag / max(desired_dir[2], 1e-6), 1e-6)
            desired_dir = np.array(
                [desired_dir[0] * scale, desired_dir[1] * scale, desired_dir[2]],
                dtype=float,
            )
            desired_dir = desired_dir / max(np.linalg.norm(desired_dir), 1e-9)

        yaw_cmd = np.arctan2(desired_dir[1], desired_dir[0])
        pitch_cmd = np.arctan2(np.hypot(desired_dir[0], desired_dir[1]), desired_dir[2])

        self.gimbal_actuator.angle_rad[:] = np.zeros(2)
        self.gimbal_actuator.rate_rad_per_s[:] = 0.0
        self.thrust_actuator.thrust_n = np.clip(
            desired_thrust_mag, 0.0, self.config.physical.max_thrust_n
        )
        
        # Initialize debug monitor
        self.debug_monitor = DebugMonitor(every_n_steps=2)
        self.current_time = 0.0
        self.state.attitude = np.array([0.0, pitch_cmd, yaw_cmd], dtype=float)

        self.time = 0.0
        self._last_mpc_result = None  # For debug output

    def step(self, dt: float) -> dict:
        pos_ref, vel_ref, acc_ref = self.trajectory.sample(self.time)

        if hasattr(self.trajectory, "time_from_position"):
            s_closest = self.trajectory.time_from_position(self.state.position)
            pos_closest, _, _ = self.trajectory.sample(s_closest)
        else:
            pos_closest = pos_ref.copy()

        # MPC reference horizon (progress-based path following)
        if self.config.mpc.use_progress_tracking and hasattr(self.trajectory, "time_from_position"):
            s0 = self.trajectory.time_from_position(self.state.position)
            p0, v0, _ = self.trajectory.sample(s0)
            ref_speed = np.linalg.norm(v0)
            if ref_speed < 1e-6:
                s_dot = 1.0
            else:
                t_hat = v0 / ref_speed
                v_along = float(np.dot(self.state.velocity, t_hat))
                s_dot = v_along / ref_speed
            s_dot = float(
                np.clip(
                    s_dot,
                    self.config.mpc.progress_min_scale,
                    self.config.mpc.progress_max_scale,
                )
            )
            # Lead the reference forward so the target is always in front
            ref_time = s0 + self.config.mpc.lead_time_s * s_dot
        else:
            # Time-based tracking: lead forward in time if possible
            if hasattr(self.trajectory, "time_from_position"):
                s0 = self.trajectory.time_from_position(self.state.position)
                ref_time = s0 + self.config.mpc.lead_time_s
            else:
                ref_time = self.time + self.config.mpc.lead_time_s
            s_dot = 1.0

        N = self.config.mpc.horizon_steps
        pos_refs = np.zeros((3, N + 1))
        vel_refs = np.zeros((3, N + 1))
        for k in range(N + 1):
            s = ref_time + s_dot * k * dt
            p_ref, v_ref, _ = self.trajectory.sample(s)
            pos_refs[:, k] = p_ref
            vel_refs[:, k] = v_ref * s_dot

        if self.config.lqr.enable_lqr:
            # LQR trajectory tracking control
            # Construct state error vector: [pos_err, vel_err, att_err, ang_rate_err]
            pos_error = self.state.position - p_ref
            vel_error = self.state.velocity - v_ref
            att_error = self.state.attitude[1:3]  # pitch and yaw errors (roll fixed at 0)
            ang_rate_error = self.state.angular_velocity[1:3]  # pitch and yaw rate errors
            
            state_error = np.concatenate([
                pos_error,      # 3 elements
                vel_error,      # 3 elements
                att_error,      # 2 elements (pitch, yaw)
                ang_rate_error  # 2 elements (pitch rate, yaw rate)
            ])
            
            # Compute optimal control input
            control_cmd = self.lqr.compute_control(state_error)
            gimbal_pitch_cmd = np.clip(control_cmd[0], -deg2rad(self.config.actuators.gimbal_max_deg), 
                                                        deg2rad(self.config.actuators.gimbal_max_deg))
            gimbal_yaw_cmd = np.clip(control_cmd[1], -deg2rad(self.config.actuators.gimbal_max_deg),
                                                     deg2rad(self.config.actuators.gimbal_max_deg))
            thrust_cmd = np.clip(control_cmd[2], 0.0, self.config.physical.max_thrust_n)
            
            # For MPC result tracking (compatibility with existing logging)
            desired_acc = np.array([0.0, 0.0, 0.0])  # Will be computed from gimbal/thrust
            mpc_result = {
                "accel_cmd": desired_acc,
                "cost": 0.0,
                "status": "lqr_active",
                "predicted_pos": np.tile(self.state.position[:, None], (1, self.config.mpc.horizon_steps + 1)),
                "predicted_vel": np.zeros((3, self.config.mpc.horizon_steps + 1)),
                "ref_pos": pos_refs,
                "ref_vel": vel_refs,
                "cost_pos": 0.0,
                "cost_vel": 0.0,
                "cost_accel": 0.0,
                "cost_jerk": 0.0,
                "cost_total": 0.0
            }
            self._last_mpc_result = mpc_result
            
        elif self.config.mpc.enable_mpc:
            mpc_result = self.mpc.solve(
                position=self.state.position,
                velocity=self.state.velocity,
                x_ref=pos_refs,
                v_ref=vel_refs,
            )
            
            desired_acc = mpc_result["accel_cmd"]
            gimbal_pitch_cmd = 0.0
            gimbal_yaw_cmd = 0.0
            thrust_cmd = 0.0
        else:
            # STABLE BASELINE: Pure gravity compensation + stabilize attitude
            desired_acc = np.array([0.0, 0.0, 0.0])
            mpc_result = {
                "accel_cmd": desired_acc,
                "cost": 0.0,
                "status": "disabled",
                "predicted_pos": np.tile(self.state.position[:, None], (1, self.config.mpc.horizon_steps + 1)),
                "predicted_vel": np.zeros((3, self.config.mpc.horizon_steps + 1)),
                "ref_pos": pos_refs,
                "ref_vel": vel_refs,
                "cost_pos": 0.0,
                "cost_vel": 0.0,
                "cost_accel": 0.0,
                "cost_jerk": 0.0,
                "cost_total": 0.0
            }
            self._last_mpc_result = mpc_result
            gimbal_pitch_cmd = 0.0
            gimbal_yaw_cmd = 0.0
            thrust_cmd = 0.0

        # Desired thrust vector (including gravity compensation)
        if self.config.lqr.enable_lqr:
            # Use LQR's desired attitude and thrust directly
            desired_thrust_mag = np.clip(thrust_cmd, 0.0, self.config.physical.max_thrust_n)
            desired_thrust_dir = direction_from_angles(self.lqr.desired_pitch, self.lqr.desired_yaw)
            # For logging consistency
            desired_acc = (desired_thrust_mag / self.mass) * desired_thrust_dir + self.gravity
        elif self.config.mpc.enable_mpc:
            # Compute from acceleration command
            desired_thrust_vec = self.mass * (desired_acc - self.gravity)
            desired_thrust_mag = np.linalg.norm(desired_thrust_vec)
            desired_thrust_mag = np.clip(desired_thrust_mag, 0.0, self.config.physical.max_thrust_n)

            if desired_thrust_mag < 1e-6:
                desired_thrust_dir = np.array([0.0, 0.0, 1.0])
            else:
                desired_thrust_dir = desired_thrust_vec / desired_thrust_mag
        else:
            # Use commanded values directly
            desired_thrust_mag = thrust_cmd
            desired_thrust_dir = np.array([0.0, 0.0, 1.0])

        # Desired body attitude: align with desired THRUST direction to provide acceleration
        # The body should point in the direction needed to achieve the desired acceleration
        desired_body_dir = desired_thrust_dir
        
        desired_pitch, desired_yaw = pitch_yaw_from_direction(desired_body_dir)
        
        # DEBUG: Log what the desired direction is
        if not self.config.mpc.enable_mpc:
            # When MPC disabled: desired direction should be straight up [0,0,1]
            # This should give desired_pitch = 0, desired_yaw = 0 (or indeterminate)
            pass

        # Body attitude response (first-order, rate limited)
        # Target: pitch/yaw to align with thrust direction, ROLL MUST BE ZERO (stabilized)
        # Unwrap yaw to nearest equivalent to prevent discontinuity
        yaw_error = desired_yaw - self.state.attitude[2]
        # Normalize yaw error to [-pi, pi]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        
        # Pitch compensation will be added later after gimbal command is computed
        
        att_error = np.array([
            -self.state.attitude[0],      # Roll error: drive to zero
            desired_pitch - self.state.attitude[1],  # Pitch error
            yaw_error                      # Yaw error (normalized)
        ])
        
        # Attitude Control: SIMPLE PROPORTIONAL on ATTITUDE ERROR (iteration 8 stable)
        # Goal: drive attitude error (err) to zero by commanding angular velocity
        tau = max(self.config.actuators.body_time_constant_s, 1e-3)
        
        # Proportional gain: converts attitude error directly to desired angular velocity
        # This is the STABLE version: no derivative on attitude, no rate error feedback
        Kp_att = 1.0 / tau  # Full proportional gain
        
        # Attitude rate command = proportional only
        att_rate_cmd = Kp_att * att_error
        max_rate = deg2rad(self.config.actuators.body_rate_limit_deg_per_s)
        att_rate_cmd[:] = np.clip(att_rate_cmd[:], -max_rate, max_rate)

        # Update attitude from angular velocity
        self.state.attitude += self.state.angular_velocity * dt
        
        # Wrap roll and yaw to [-pi, pi], clamp pitch to [-pi/2, pi/2]
        self.state.attitude[0] = np.arctan2(np.sin(self.state.attitude[0]), np.cos(self.state.attitude[0]))  # Roll
        self.state.attitude[2] = np.arctan2(np.sin(self.state.attitude[2]), np.cos(self.state.attitude[2]))  # Yaw
        self.state.attitude[1] = np.clip(self.state.attitude[1], -np.pi/2, np.pi/2)

        # Compute gimbal command relative to body
        R_body = rotation_matrix_from_pitch_yaw(self.state.attitude[1], self.state.attitude[2])
        
        if self.config.lqr.enable_lqr:
            # Use LQR-computed gimbal commands directly
            gimbal_cmd = np.array([gimbal_pitch_cmd, gimbal_yaw_cmd], dtype=float)

            # Add roll stabilization using gimbal yaw (same approach as non-LQR)
            roll_error = -self.state.attitude[0]
            roll_rate = self.state.angular_velocity[0]
            gimbal_yaw_roll_control = 0.3 * roll_error + 0.15 * roll_rate
            gimbal_cmd[1] = np.clip(gimbal_cmd[1] + gimbal_yaw_roll_control, -0.262, 0.262)
        else:
            desired_thrust_dir_body = R_body.T @ desired_thrust_dir
            gimbal_pitch, gimbal_yaw = pitch_yaw_from_direction(desired_thrust_dir_body)
            
            # When MPC is disabled (gravity compensation mode), zero out gimbal to avoid fighting attitude control
            if not self.config.mpc.enable_mpc:
                gimbal_pitch = 0.0
                gimbal_yaw = 0.0
            # Roll control: use gimbal yaw to counteract roll (standard approach)
            roll_error = -self.state.attitude[0]
            roll_rate = self.state.angular_velocity[0]
            
            # Simple proportional + damping for roll stabilization
            gimbal_yaw_roll_control = 0.3 * roll_error + 0.15 * roll_rate  # Conservative gains
            
            # Combine trajectory yaw with roll control
            gimbal_yaw_limited = gimbal_yaw + gimbal_yaw_roll_control
            gimbal_yaw_limited = np.clip(gimbal_yaw_limited, -0.262, 0.262)  # ±15° max for roll control
            
            gimbal_cmd = np.array([gimbal_pitch, gimbal_yaw_limited], dtype=float)
        # Actuator dynamics
        thrust_n = self.thrust_actuator.update(desired_thrust_mag, dt)
        # Apply gimbal command through actuator dynamics
        gimbal_angles = self.gimbal_actuator.update(gimbal_cmd, dt)

        # Dynamics update (thrust direction is body orientation plus gimbal deflection)
        gimbal_dir_body = direction_from_angles(gimbal_angles[0], gimbal_angles[1])
        
        # Account for gimbal offset from center of mass creating a torque
        # Gimbal is at -gimbal_offset along body Z axis
        gimbal_pos_body = np.array([0.0, 0.0, -self.gimbal_offset])
        thrust_dir = R_body @ gimbal_dir_body
        
        # Thrust force at gimbal location
        thrust_force = thrust_n * thrust_dir
        
        # Torque from gimbal offset: τ = r × F (where r is from CoM to gimbal)
        # In body frame: r_body = [0, 0, -gimbal_offset]
        # Thrust in body frame: F_body = thrust_n * gimbal_dir_body
        # Negate gimbal direction for correct torque direction
        F_body = thrust_n * (-gimbal_dir_body)
        torque_body = np.cross(gimbal_pos_body, F_body)
        
        # Attitude dynamics from gimbal torque
        # Using moment of inertia for thin rod: I_xy ~ m * L^2 / 3
        I_xy = self.mass * (self.gimbal_offset ** 2) / 3.0
        # For yaw (body Z-axis): use larger I_z to reduce yaw spin-up sensitivity
        I_z = self.mass * (self.gimbal_offset ** 2) / 3.0  # Same as pitch/roll for stability
        
        # Angular acceleration from torques (Euler's equations simplified)
        omega_dot = np.array([
            torque_body[0] / max(I_xy, 1e-3),
            torque_body[1] / max(I_xy, 1e-3),
            torque_body[2] / max(I_z, 1e-3)
        ], dtype=float)
        
        # Attitude command as torque feedforward: directly use commanded rate to produce torque
        # This is simpler and more stable than error feedback
        # Torque = I * alpha_desired, where alpha_desired = K * (att_rate_cmd - omega)
        # Use moderate gain to track the command without being too aggressive
        Kc_att = 0.8  # Attitude tracking gain (proportional to moment of inertia scaling)
        att_feedback = Kc_att * (att_rate_cmd - self.state.angular_velocity)
        
        # Damping to prevent oscillations (iteration 8 values)
        angular_velocity_damping = -0.3 * self.state.angular_velocity  # 30% general damping
        # Roll damping (prevent roll oscillation)
        angular_velocity_damping[0] = -0.3 * self.state.angular_velocity[0]  # 30% roll damping
        # Pitch damping (prevent pitch oscillation but allow controlled descent)
        angular_velocity_damping[1] = -0.9 * self.state.angular_velocity[1]  # 90% pitch damping
        # Yaw damping (prevent yaw oscillation)
        angular_velocity_damping[2] = -0.8 * self.state.angular_velocity[2]  # 80% yaw damping
        
        # Update angular velocity from gimbal torques + attitude feedback + damping
        self.state.angular_velocity += (omega_dot + att_feedback + angular_velocity_damping) * dt
        
        thrust_acc = (thrust_n / self.mass) * thrust_dir
        total_acc = thrust_acc + self.gravity

        self.state.velocity += total_acc * dt
        self.state.position += self.state.velocity * dt

        self.time += dt
        self.current_time = self.time

        return {
            "time": self.time,
            "position": self.state.position.copy(),
            "velocity": self.state.velocity.copy(),
            "attitude": self.state.attitude.copy(),
            "angular_velocity": self.state.angular_velocity.copy(),
            "thrust_n": thrust_n,
            "gimbal_angles": gimbal_angles.copy(),
            "pos_ref": pos_ref,
            "pos_closest": pos_closest,
            "desired_accel": desired_acc.copy(),
            "desired_pitch": desired_pitch,
            "desired_yaw": desired_yaw,
            "thrust_cmd": desired_thrust_mag,
            "gimbal_cmd": gimbal_cmd.copy(),
            "att_error": att_error.copy(),
            "att_rate_cmd": att_rate_cmd.copy(),
            "mpc_pred_pos": mpc_result["predicted_pos"].copy(),
            "mpc_ref_pos": mpc_result["ref_pos"].copy(),
            "mpc_cost_pos": mpc_result["cost_pos"],
            "mpc_cost_vel": mpc_result["cost_vel"],
            "mpc_cost_accel": mpc_result["cost_accel"],
            "mpc_cost_jerk": mpc_result["cost_jerk"],
            "mpc_cost_total": mpc_result["cost_total"],
            "ref_velocity": vel_ref,
            "ref_acceleration": acc_ref,
        }

    def run(self) -> dict:
        steps = int(self.config.sim.total_time / self.config.sim.dt)
        history = {
            "time": [],
            "position": [],
            "velocity": [],
            "attitude": [],
            "angular_velocity": [],
            "thrust_n": [],
            "gimbal_angles": [],
            "pos_ref": [],
            "pos_closest": [],
            "desired_accel": [],
            "desired_pitch": [],
            "desired_yaw": [],
            "thrust_cmd": [],
            "gimbal_cmd": [],
            "att_error": [],
            "att_rate_cmd": [],
            "mpc_pred_pos": [],
            "mpc_ref_pos": [],
            "mpc_cost_pos": [],
            "mpc_cost_vel": [],
            "mpc_cost_accel": [],
            "mpc_cost_jerk": [],
            "mpc_cost_total": [],
            "ref_velocity": [],
            "ref_acceleration": [],
        }

        for i in range(steps):
            data = self.step(self.config.sim.dt)
            for key, value in data.items():
                history[key].append(value)

            if i > 0 and data["position"][2] <= 0.0:
                if self.config.debug.show_progress:
                    print("Simulation stopped: rocket hit the ground.")
                break

            if self.config.debug.show_progress and (i + 1) % self.config.debug.progress_every_steps == 0:
                progress = 100.0 * (i + 1) / max(steps, 1)
                print(f"Simulation progress: {progress:5.1f}%")
            
            # Comprehensive debug output
            debug_info = attach_debug_info(self, data)
            self.debug_monitor.log_step(debug_info)

            if self.config.debug.show_mpc_debug and (i + 1) % self.config.debug.mpc_debug_every_steps == 0:
                pos_err = np.linalg.norm(data["position"] - data["pos_ref"])
                path_err = np.linalg.norm(data["position"] - data["pos_closest"])
                speed = np.linalg.norm(data["velocity"])
                gimbal_deg = np.rad2deg(data["gimbal_angles"])
                angular_vel_rad = data["angular_velocity"]
                angular_vel_deg = np.rad2deg(angular_vel_rad)
                attitude_deg = np.rad2deg(data["attitude"])
                
                # Compute expected pointing direction along path
                mpc_ref_pos = data["mpc_ref_pos"]
                if mpc_ref_pos.shape[1] > 1:
                    v_ref = mpc_ref_pos[:, 1] - mpc_ref_pos[:, 0]
                else:
                    v_ref = np.array([0, 0, 1])
                
                if np.linalg.norm(v_ref) > 1e-6:
                    expected_dir = v_ref / np.linalg.norm(v_ref)
                else:
                    expected_dir = np.array([0, 0, 1])
                
                # Actual body pointing direction (Z-axis in body frame)
                actual_dir = np.array([0, 0, 1])
                R_body = rotation_matrix_from_pitch_yaw(attitude_deg[1], attitude_deg[2])
                actual_dir_world = R_body @ actual_dir
                
                # Angle between actual pointing and path direction
                pointing_error = np.arccos(np.clip(np.dot(actual_dir_world, expected_dir), -1, 1))
                
                roll_deg = attitude_deg[0]
                roll_rate_deg = angular_vel_deg[0]
                pitch_rate_deg = angular_vel_deg[1]
                yaw_rate_deg = angular_vel_deg[2]
                
                roll_warn = " ROLL_WARN" if abs(roll_deg) > 15 else ""
                osc_warn = " OSC_WARN" if (abs(pitch_rate_deg) > 5 or abs(yaw_rate_deg) > 5) else ""
                
                print(
                    f"t={data['time']:6.2f}s | "
                    f"err_path={path_err:6.2f}m err_ref={pos_err:6.2f}m | "
                    f"point_err={np.rad2deg(pointing_error):6.1f}° | "
                    f"speed={speed:6.2f}m/s | thrust={data['thrust_n']:5.0f}N | "
                    f"gimbal(p,y)={gimbal_deg[0]:5.1f},{gimbal_deg[1]:5.1f}° | "
                    f"att(r,p,y)={roll_deg:6.1f},{attitude_deg[1]:6.1f},{attitude_deg[2]:6.1f}° | "
                    f"w(r,p,y)={roll_rate_deg:6.1f},{pitch_rate_deg:6.1f},{yaw_rate_deg:6.1f}deg/s"
                    f"{roll_warn}{osc_warn}"
                )

        for key in history:
            history[key] = np.array(history[key])

        return history
