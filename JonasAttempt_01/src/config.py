from dataclasses import dataclass, field
import numpy as np


@dataclass
class PhysicalConfig:
    gravity: float = 9.81
    mass_kg: float = 50.0
    max_thrust_n: float = 600.0  # Reduced from 2000 to ~1.2x weight for hover capability
    gimbal_offset_m: float = 3.5     # Distance from CoM to engine along body axis


@dataclass
class ActuatorConfig:
    # Thrust actuator dynamics (first-order lag + rate limit)
    # Typical throttle response times for small liquid engines are on the order of 0.2–0.5 s.
    thrust_time_constant_s: float = 0.35
    thrust_rate_limit_n_per_s: float = 8000.0

    # Gimbal limits - ±5° (iteration 8 stable baseline)
    gimbal_max_deg: float = 5.0  # Conservative authority for stability
    gimbal_rate_limit_deg_per_s: float = 20.0  # Original
    gimbal_accel_limit_deg_per_s2: float = 100.0  # Original

    # Body attitude response (simplified rotational dynamics)
    body_time_constant_s: float = 0.8  # Normal attitude response
    body_rate_limit_deg_per_s: float = 40.0  # Reduced from 50


@dataclass
class ControllerConfig:
    # Legacy PD gains (unused when MPC is enabled)
    pos_kp: float = 3.0
    pos_kd: float = 2.5
    max_desired_accel: float = 20.0


@dataclass
class LQRConfig:
    """Cascade controller tuning for trajectory tracking"""
    # Position loop (outer)
    weight_pos_error: float = 0.05      # Position → velocity correction gain (m/s per m)
    weight_pos_integral: float = 0.01   # Position integral gain
    # Velocity loop (middle)
    weight_vel_error: float = 0.03      # Velocity → attitude setpoint gain (rad per m/s)
    weight_vel_integral: float = 0.005  # Velocity integral gain (rad per (m/s))
    # Attitude loop (inner)
    weight_att_error: float = 1.2       # Attitude proportional gain (rad per rad error)
    weight_ang_rate_error: float = 0.3  # Angular rate damping (rad per rad/s)
    lead_time_s: float = 0.08           # Phase lead time constant (s)
    # Thrust loop
    weight_thrust: float = 12.0         # Vertical tracking gain (N per m/s)
    weight_thrust_integral: float = 0.8 # Vertical integral gain (N per m)
    # Actuator limits
    gimbal_max_deg: float = 5.0        # Gimbal angle limit (iteration 8 stable)
    gimbal_rate_deg_s: float = 20.0     # Gimbal rate limit
    # Attitude limits
    max_pitch_deg: float = 15.0
    max_yaw_deg: float = 10.0
    # Reference governor limits
    max_vel_cmd: float = 12.0           # m/s per axis
    max_vel_cmd_rate: float = 6.0       # m/s^2 per axis
    # Integrator limits
    int_pos_limit: float = 50.0         # m*s
    int_vel_limit: float = 30.0         # (m/s)*s
    # State filtering
    filter_alpha: float = 0.2           # 0..1 (higher = less filtering)
    # Timing
    dt: float = 0.01                    # Simulation timestep
    enable_lqr: bool = True             # Use cascade controller


@dataclass
class MPCConfig:
    horizon_steps: int = 15
    weight_pos: float = 1.0
    weight_vel: float = 0.5
    weight_accel: float = 50.0
    weight_lateral_accel: float = 100.0
    weight_jerk_thrust: float = 30.0
    max_accel: float = 3.0
    max_lateral_accel: float = 0.15
    use_progress_tracking: bool = True
    progress_min_scale: float = 0.2
    progress_max_scale: float = 2.0
    lead_time_s: float = 0.0
    enable_mpc: bool = False  # DISABLED - using LQR instead


@dataclass
class SimulationConfig:
    dt: float = 0.02
    total_time: float = 150  # Extended to 150s for full descent + landing

    # Trajectory parameters
    trajectory_type: str = "parabolic_cruise"
    trajectory_radius_m: float = 30.0
    trajectory_height_m: float = 160.0
    trajectory_turns: float = 1.5
    curve_amp_m: float = 2.0

    # Parabolic cruise trajectory - SIMPLE: near-vertical hover then soft descent
    # Phase 1 (0-40s): Ascend slowly to 500m
    # Phase 2 (40-50s): Coast at altitude
    # Phase 3 (50-150s): Slow descent at 2 m/s
    parabola_peak_m: float = 500.0       # Peak altitude: 500m
    parabola_ascent_length_m: float = 400.0   # Vertical ascent 0-40s
    parabola_cruise_length_m: float = 100.0   # Coast 40-50s
    parabola_descent_length_m: float = 400.0  # Slow descent 50-150s at 2 m/s
    parabola_ascent_time_s: float = 40.0
    parabola_cruise_time_s: float = 10.0      # Short coast phase
    parabola_descent_time_s: float = 100.0    # Long slow descent


@dataclass
class VisualizationConfig:
    trail_length: int = 500
    axis_margin: float = 5.0
    
    # Performance settings
    update_interval: int = 2          # Update every N sim steps
    trail_downsample_stride: int = 2  # Keep every Nth trail point
    mesh_quality: int = 8             # Circumferential points (8/12/16)
    
    # Layout settings
    show_rocket_closeup: bool = True
    show_mpc_internals: bool = True
    mpc_horizon_projection: str = "xz"  # "xy" or "xz"
    cost_window_steps: int = 200      # Rolling window for cost plots


@dataclass
class DebugConfig:
    show_progress: bool = True
    progress_every_steps: int = 1     # Every step for detailed output
    show_mpc_debug: bool = True
    mpc_debug_every_steps: int = 1    # Every step for detailed output
    show_path_following: bool = True  # Distance to path, pointing direction
    show_gimbal_torque: bool = True   # Gimbal-induced torques and angular rates


@dataclass
class Config:
    physical: PhysicalConfig = field(default_factory=PhysicalConfig)
    actuators: ActuatorConfig = field(default_factory=ActuatorConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    lqr: LQRConfig = field(default_factory=LQRConfig)
    mpc: MPCConfig = field(default_factory=MPCConfig)
    sim: SimulationConfig = field(default_factory=SimulationConfig)
    viz: VisualizationConfig = field(default_factory=VisualizationConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)


CONFIG = Config()


def deg2rad(deg: float) -> float:
    return np.deg2rad(deg)


def rad2deg(rad: float) -> float:
    return np.rad2deg(rad)
