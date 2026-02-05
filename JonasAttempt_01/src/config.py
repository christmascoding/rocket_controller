from dataclasses import dataclass, field
import numpy as np


@dataclass
class PhysicalConfig:
    gravity: float = 9.81
    mass_kg: float = 50.0
    max_thrust_n: float = 2000.0


@dataclass
class ActuatorConfig:
    # Thrust actuator dynamics (first-order lag + rate limit)
    # Typical throttle response times for small liquid engines are on the order of 0.2–0.5 s.
    thrust_time_constant_s: float = 0.35
    thrust_rate_limit_n_per_s: float = 8000.0

    # Gimbal limits (semi-realistic for small rockets)
    gimbal_max_deg: float = 15.0
    gimbal_rate_limit_deg_per_s: float = 60.0
    gimbal_accel_limit_deg_per_s2: float = 300.0

    # Body attitude response (simplified rotational dynamics)
    body_time_constant_s: float = 0.4
    body_rate_limit_deg_per_s: float = 70.0


@dataclass
class ControllerConfig:
    # Legacy PD gains (unused when MPC is enabled)
    pos_kp: float = 3.0
    pos_kd: float = 2.5
    max_desired_accel: float = 20.0


@dataclass
class MPCConfig:
    horizon_steps: int = 15
    weight_pos: float = 30.0
    weight_vel: float = 15.0
    weight_accel: float = 5.0
    weight_jerk_thrust: float = 10.0
    max_accel: float = 12.0
    max_lateral_accel: float = 2.5
    use_progress_tracking: bool = True
    progress_min_scale: float = 0.2
    progress_max_scale: float = 2.0


@dataclass
class SimulationConfig:
    dt: float = 0.02
    total_time: float = 90

    # Trajectory parameters
    trajectory_type: str = "parabolic_cruise"
    trajectory_radius_m: float = 30.0
    trajectory_height_m: float = 160.0
    trajectory_turns: float = 1.5
    curve_amp_m: float = 2.0

    # Parabolic cruise trajectory parameters
    parabola_peak_m: float = 600.0
    parabola_ascent_length_m: float = 1000.0
    parabola_cruise_length_m: float = 700.0
    parabola_descent_length_m: float = 2000.0
    parabola_ascent_time_s: float = 45.0
    parabola_cruise_time_s: float = 15.0
    parabola_descent_time_s: float = 32.0


@dataclass
class VisualizationConfig:
    trail_length: int = 500
    axis_margin: float = 5.0


@dataclass
class DebugConfig:
    show_progress: bool = True
    progress_every_steps: int = 25
    show_mpc_debug: bool = True
    mpc_debug_every_steps: int = 25


@dataclass
class Config:
    physical: PhysicalConfig = field(default_factory=PhysicalConfig)
    actuators: ActuatorConfig = field(default_factory=ActuatorConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    mpc: MPCConfig = field(default_factory=MPCConfig)
    sim: SimulationConfig = field(default_factory=SimulationConfig)
    viz: VisualizationConfig = field(default_factory=VisualizationConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)


CONFIG = Config()


def deg2rad(deg: float) -> float:
    return np.deg2rad(deg)


def rad2deg(rad: float) -> float:
    return np.rad2deg(rad)
