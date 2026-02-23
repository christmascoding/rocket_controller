"""
Cascade Control Architecture for Rocket Trajectory Tracking

Three-level hierarchical control:
1. OUTER LOOP: Position error → Reference velocity (P controller, slow ~2s)
2. MIDDLE LOOP: Velocity error → Reference attitude (P controller, medium ~0.5s) 
3. INNER LOOP: Attitude error → Gimbal rate command (PD controller, fast with rate limiting)
4. THRUST: Vertical velocity → Thrust (P controller)

Key design principle: Each loop operates at its natural frequency, accounting for 
actuator rate limits. Gimbal rate limit (20°/s) is INTEGRATED INTO the control law,
not fought against.
"""

import numpy as np

from src.config import LQRConfig


class LQRTrajectoryController:
    """Cascade control architecture with rate-limited actuators."""
    
    def __init__(self, config: LQRConfig):
        self.config = config
        self.int_pos = np.zeros(3)
        self.int_vel = np.zeros(3)
        self.prev_vel_cmd = np.zeros(3)
        self.filt_vel = np.zeros(3)
        self.filt_att = np.zeros(2)
        self.filt_rate = np.zeros(2)
        print(f"[Controller] Initialized cascade control (3-level hierarchy)")
    
    def compute_control(self, state_error: np.ndarray) -> np.ndarray:
        """
        Compute control using cascade architecture with rate-limited gimbal.
        
        The KEY insight: Gimbal has 20°/s rate limit. Don't command it directly.
        Instead: Command gimbal RATE, limited to physical actuator capability.
        
        Args:
            state_error: [pos_err_x, pos_err_y, pos_err_z, 
                         vel_err_x, vel_err_y, vel_err_z,
                         att_err_pitch, att_err_yaw,
                         ang_rate_err_pitch, ang_rate_err_yaw]
        
        Returns:
            [gimbal_pitch_cmd, gimbal_yaw_cmd, thrust_cmd] (degrees and Newtons)
        """
        # Extract state errors (state_error is actual - reference)
        pos_err = state_error[0:3]
        vel_err = state_error[3:6]
        att_meas = state_error[6:8]
        ang_rate_meas = state_error[8:10]
        
        m = 50.0
        g = 9.81
        gimbal_max = self.config.gimbal_max_deg
        gimbal_rate_limit = self.config.gimbal_rate_deg_s  # °/s
        
        # Convert errors to tracking form (reference - actual)
        e_pos = -pos_err
        e_vel = -vel_err

        # State filtering (simple alpha filter)
        alpha = np.clip(self.config.filter_alpha, 0.0, 1.0)
        self.filt_vel = alpha * vel_err + (1.0 - alpha) * self.filt_vel
        self.filt_att = alpha * att_meas + (1.0 - alpha) * self.filt_att
        self.filt_rate = alpha * ang_rate_meas + (1.0 - alpha) * self.filt_rate

        # Work entirely in radians (controller outputs radians)
        pitch_rad = self.filt_att[0]
        yaw_rad = self.filt_att[1]
        pitch_rate_rad = self.filt_rate[0]
        yaw_rate_rad = self.filt_rate[1]

        # ========== OUTER LOOP: Position → Velocity Correction ==========
        # Low gain, slow response to position error with integral action
        kp_pos = self.config.weight_pos_error
        ki_pos = self.config.weight_pos_integral
        self.int_pos += e_pos * self.config.dt
        self.int_pos = np.clip(self.int_pos, -self.config.int_pos_limit, self.config.int_pos_limit)
        vel_cmd_raw = kp_pos * e_pos + ki_pos * self.int_pos

        # Reference governor: limit velocity command magnitude and rate
        vel_cmd_limited = np.clip(vel_cmd_raw, -self.config.max_vel_cmd, self.config.max_vel_cmd)
        max_delta = self.config.max_vel_cmd_rate * self.config.dt
        vel_cmd = self.prev_vel_cmd + np.clip(vel_cmd_limited - self.prev_vel_cmd, -max_delta, max_delta)
        self.prev_vel_cmd = vel_cmd
        
        # ========== MIDDLE LOOP: Velocity → Attitude Setpoint ==========
        # Convert velocity tracking error to desired pitch/yaw attitude
        kp_vel = self.config.weight_vel_error
        ki_vel = self.config.weight_vel_integral
        vel_track = e_vel + vel_cmd
        self.int_vel += vel_track * self.config.dt
        self.int_vel = np.clip(self.int_vel, -self.config.int_vel_limit, self.config.int_vel_limit)

        max_pitch_rad = np.deg2rad(self.config.max_pitch_deg)
        max_yaw_rad = np.deg2rad(self.config.max_yaw_deg)
        # Desired attitude from lateral velocity tracking
        desired_pitch = np.clip((kp_vel * vel_track[0] + ki_vel * self.int_vel[0]), -max_pitch_rad, max_pitch_rad)
        desired_yaw = np.clip((kp_vel * vel_track[1] + ki_vel * self.int_vel[1]), -max_yaw_rad, max_yaw_rad)

        # Expose desired attitude for simulator alignment
        self.desired_pitch = desired_pitch
        self.desired_yaw = desired_yaw
        
        # ========== INNER LOOP: Attitude → Gimbal Angle Command ==========
        # Use PD control on attitude error to command gimbal ANGLE.
        # The actuator model enforces rate/accel limits internally.
        kp_att = self.config.weight_att_error
        kd_att = self.config.weight_ang_rate_error
        
        # Attitude errors (radians)
        att_pitch_err = desired_pitch - pitch_rad
        att_yaw_err = desired_yaw - yaw_rad

        # Lead compensation (phase advance): e_lead = e + T * e_dot, with e_dot ≈ -rate
        lead_t = self.config.lead_time_s
        att_pitch_err = att_pitch_err - lead_t * pitch_rate_rad
        att_yaw_err = att_yaw_err - lead_t * yaw_rate_rad
        
        # Gimbal angle command (radians) from PD controller
        # Invert gimbal sign to match torque direction (gimbal deflection produces opposite moment)
        gimbal_pitch_cmd = -(kp_att * att_pitch_err - kd_att * pitch_rate_rad)
        gimbal_yaw_cmd = -(kp_att * att_yaw_err - kd_att * yaw_rate_rad)
        
        # Saturate gimbal to angle limits
        gimbal_max_rad = np.deg2rad(gimbal_max)
        gimbal_pitch_cmd = np.clip(gimbal_pitch_cmd, -gimbal_max_rad, gimbal_max_rad)
        gimbal_yaw_cmd = np.clip(gimbal_yaw_cmd, -gimbal_max_rad, gimbal_max_rad)

        # Anti-windup: freeze integrators if saturated
        if abs(gimbal_pitch_cmd) >= 0.98 * gimbal_max_rad:
            self.int_vel[0] *= 0.98
        if abs(gimbal_yaw_cmd) >= 0.98 * gimbal_max_rad:
            self.int_vel[1] *= 0.98
        
        # ========== THRUST LOOP: Vertical Velocity → Thrust ==========
        # Proportional control on vertical tracking error (velocity + position)
        kp_thrust = self.config.weight_thrust
        thrust_hover = m * g  # ~490N
        ki_thrust = self.config.weight_thrust_integral
        thrust_cmd = thrust_hover + kp_thrust * vel_track[2] + ki_thrust * self.int_pos[2]
        
        # Saturate thrust
        thrust_cmd = np.clip(thrust_cmd, 0.0, 600.0)

        # Anti-windup for vertical integrator
        if thrust_cmd <= 0.0 or thrust_cmd >= 600.0:
            self.int_pos[2] *= 0.98
        
        return np.array([gimbal_pitch_cmd, gimbal_yaw_cmd, thrust_cmd])
