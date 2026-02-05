from dataclasses import dataclass
import numpy as np

from src.config import Config, deg2rad
from src.controllers.mpc import MPCController, MPCParams
from src.models.actuators import ThrustActuator, GimbalActuator
from src.models.trajectory import HelixTrajectory, UpwardCurveTrajectory, ParabolicCruiseTrajectory


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

        self.mpc = MPCController(
            MPCParams(
                horizon_steps=config.mpc.horizon_steps,
                dt=config.sim.dt,
                weight_pos=config.mpc.weight_pos,
                weight_vel=config.mpc.weight_vel,
                weight_accel=config.mpc.weight_accel,
                weight_jerk_thrust=config.mpc.weight_jerk_thrust,
                max_accel=config.mpc.max_accel,
                max_lateral_accel=config.mpc.max_lateral_accel,
            )
        )

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
        self.state.attitude = np.array([0.0, pitch_cmd, yaw_cmd], dtype=float)

        self.time = 0.0

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
            ref_time = s0
        else:
            ref_time = self.time
            s_dot = 1.0

        N = self.config.mpc.horizon_steps
        pos_refs = np.zeros((3, N + 1))
        vel_refs = np.zeros((3, N + 1))
        for k in range(N + 1):
            s = ref_time + s_dot * k * dt
            p_ref, v_ref, _ = self.trajectory.sample(s)
            pos_refs[:, k] = p_ref
            vel_refs[:, k] = v_ref * s_dot

        desired_acc = self.mpc.solve(
            position=self.state.position,
            velocity=self.state.velocity,
            x_ref=pos_refs,
            v_ref=vel_refs,
        )

        # Desired thrust vector (including gravity compensation)
        desired_thrust_vec = self.mass * (desired_acc - self.gravity)
        desired_thrust_mag = np.linalg.norm(desired_thrust_vec)
        desired_thrust_mag = np.clip(desired_thrust_mag, 0.0, self.config.physical.max_thrust_n)

        if desired_thrust_mag < 1e-6:
            desired_dir = np.array([0.0, 0.0, 1.0])
        else:
            desired_dir = desired_thrust_vec / desired_thrust_mag

        # Desired body attitude to align with desired thrust direction
        desired_pitch, desired_yaw = pitch_yaw_from_direction(desired_dir)

        # Body attitude response (first-order, rate limited)
        att_error = np.array([0.0, desired_pitch, desired_yaw]) - self.state.attitude
        tau = max(self.config.actuators.body_time_constant_s, 1e-3)
        att_rate_cmd = att_error / tau
        max_rate = deg2rad(self.config.actuators.body_rate_limit_deg_per_s)
        att_rate_cmd[1:] = np.clip(att_rate_cmd[1:], -max_rate, max_rate)

        self.state.attitude += att_rate_cmd * dt
        self.state.angular_velocity = att_rate_cmd.copy()

        # Compute gimbal command relative to body to achieve desired thrust
        R_body = rotation_matrix_from_pitch_yaw(self.state.attitude[1], self.state.attitude[2])
        desired_dir_body = R_body.T @ desired_dir
        gimbal_pitch, gimbal_yaw = pitch_yaw_from_direction(desired_dir_body)
        gimbal_cmd = np.array([gimbal_pitch, gimbal_yaw], dtype=float)

        # Actuator dynamics
        thrust_n = self.thrust_actuator.update(desired_thrust_mag, dt)
        gimbal_angles = self.gimbal_actuator.update(gimbal_cmd, dt)

        # Dynamics update (thrust direction is body orientation plus gimbal deflection)
        gimbal_dir_body = direction_from_angles(gimbal_angles[0], gimbal_angles[1])
        thrust_dir = R_body @ gimbal_dir_body
        thrust_acc = (thrust_n / self.mass) * thrust_dir
        total_acc = thrust_acc + self.gravity

        self.state.velocity += total_acc * dt
        self.state.position += self.state.velocity * dt

        self.time += dt

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

            if self.config.debug.show_mpc_debug and (i + 1) % self.config.debug.mpc_debug_every_steps == 0:
                pos_err = np.linalg.norm(data["position"] - data["pos_ref"])
                speed = np.linalg.norm(data["velocity"])
                gimbal_deg = np.rad2deg(data["gimbal_angles"])
                print(
                    "MPC debug | "
                    f"t={data['time']:.2f}s "
                    f"pos_err={pos_err:.2f}m "
                    f"speed={speed:.2f}m/s "
                    f"thrust={data['thrust_n']:.0f}N "
                    f"gimbal(p,y)=({gimbal_deg[0]:.1f},{gimbal_deg[1]:.1f})deg"
                )

        for key in history:
            history[key] = np.array(history[key])

        return history
