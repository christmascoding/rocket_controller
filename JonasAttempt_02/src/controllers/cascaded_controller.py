"""
Cascaded Trajectory Tracking Controller
Implements proper aerospace control architecture with three nested loops.

Architecture:
1. OUTER LOOP (Guidance):     Position error → Desired acceleration vector
2. MIDDLE LOOP (Attitude):    Desired acceleration → Desired orientation + throttle
3. INNER LOOP (Stabilization): Orientation error → Gimbal angles
"""

import numpy as np
from src.config import RocketSpecs


class GuidanceController:
    """
    Outer Loop: Computes desired acceleration to track the reference trajectory.
    
    Uses PD control to drive position and velocity errors to zero.
    Output is a desired acceleration vector in the inertial frame.
    """
    
    def __init__(self, Kp=None, Kd=None):
        """
        Initialize guidance controller with position and velocity gains.
        
        Args:
            Kp: Position gain matrix (diagonal 3x3), default [0.001, 0.001, 0.01]
            Kd: Velocity gain matrix (diagonal 3x3), default [0.02, 0.02, 0.05]
        """
        self.Kp = np.array(Kp) if Kp is not None else np.array([0.001, 0.001, 0.01])
        self.Kd = np.array(Kd) if Kd is not None else np.array([0.02, 0.02, 0.05])
        self.gravity = 9.81
    
    def compute_desired_acceleration(self, actual_pos, actual_vel, desired_pos, desired_vel):
        """
        Compute desired acceleration vector to track reference trajectory.
        
        a_desired = -Kp*(pos - pos_desired) - Kd*(vel - vel_desired) + [0, 0, -g]
        
        Args:
            actual_pos: Current position [x, y, z]
            actual_vel: Current velocity [vx, vy, vz]
            desired_pos: Desired position [x, y, z]
            desired_vel: Desired velocity [vx, vy, vz]
        
        Returns:
            Desired acceleration vector in inertial frame
        """
        pos_error = actual_pos - desired_pos
        vel_error = actual_vel - desired_vel
        
        # Gravity acts downward
        gravity_accel = np.array([0.0, 0.0, -self.gravity])
        
        # Desired acceleration from tracking error
        desired_accel = gravity_accel - self.Kp * pos_error - self.Kd * vel_error
        
        return desired_accel


class AttitudeController:
    """
    Middle Loop: Converts desired acceleration into desired attitude and throttle.
    
    The rocket engine produces thrust along its body Z-axis (after gimbal rotation).
    We compute what pointing direction is needed to achieve the desired acceleration.
    """
    
    def __init__(self, max_thrust=300_000.0, rocket_mass=50_000.0):
        """
        Initialize attitude controller.
        
        Args:
            max_thrust: Maximum engine thrust in Newtons
            rocket_mass: Rocket mass in kg
        """
        self.max_thrust = max_thrust
        self.rocket_mass = rocket_mass
        self.gravity = 9.81
    
    def compute_desired_attitude(self, desired_accel):
        """
        Compute desired pitch, yaw angles and throttle to achieve desired acceleration.
        
        The rocket's thrust vector must overcome gravity and provide the desired acceleration.
        
        Args:
            desired_accel: Desired acceleration vector [ax, ay, az]
        
        Returns:
            (throttle, desired_pitch, desired_yaw)
            - throttle: Normalized thrust (0.0 to 1.0)
            - desired_pitch: Pitch angle in radians
            - desired_yaw: Yaw angle in radians
        """
        # Required thrust acceleration vector (subtract gravity)
        required_thrust_accel = desired_accel - np.array([0.0, 0.0, -self.gravity])
        
        # Required thrust magnitude and direction
        required_accel_mag = np.linalg.norm(required_thrust_accel)
        max_accel = self.max_thrust / self.rocket_mass
        
        # Compute throttle (clamp to [0, 1])
        if required_accel_mag > 0.1:
            throttle = required_accel_mag / max_accel
            throttle = np.clip(throttle, 0.0, 1.0)
            
            # Desired thrust direction (unit vector)
            desired_thrust_dir = required_thrust_accel / required_accel_mag
        else:
            # Very small required acceleration - minimal throttle
            throttle = 0.1
            desired_thrust_dir = np.array([0.0, 0.0, 1.0])
        
        # Convert thrust direction to pitch and yaw angles
        # For ZYX (yaw-pitch-roll), body +Z axis in inertial is:
        # [cos(yaw)*sin(pitch), sin(yaw)*sin(pitch), cos(pitch)]
        desired_pitch = np.arctan2(
            np.sqrt(desired_thrust_dir[0]**2 + desired_thrust_dir[1]**2),
            desired_thrust_dir[2]
        )
        desired_yaw = np.arctan2(desired_thrust_dir[1], desired_thrust_dir[0])
        
        return throttle, desired_pitch, desired_yaw


class StabilizationController:
    """
    Inner Loop: Computes gimbal angles to achieve desired attitude.
    
    Uses PD control on orientation error to dampen rotations and track desired attitude.
    """
    
    def __init__(self, Kp=0.5, Kd=0.15, gimbal_max=None):
        """
        Initialize stabilization controller.
        
        Args:
            Kp: Proportional gain for angle error
            Kd: Derivative gain for angular velocity (damping)
            gimbal_max: Maximum gimbal deflection angle (radians)
        """
        self.Kp = Kp
        self.Kd = Kd
        self.gimbal_max = gimbal_max or np.radians(15.0)
    
    def compute_gimbal_angles(self, current_pitch, current_yaw, 
                             current_pitch_rate, current_yaw_rate,
                             desired_pitch, desired_yaw):
        """
        Compute gimbal angles to track desired attitude.
        
        Uses PD control:
        gimbal = -Kp*angle_error - Kd*angle_rate
        
        Args:
            current_pitch, current_yaw: Current Euler angles (radians)
            current_pitch_rate, current_yaw_rate: Angular velocity (rad/s)
            desired_pitch, desired_yaw: Desired Euler angles (radians)
        
        Returns:
            (gimbal_pitch, gimbal_yaw) in radians
        """
        # Angle errors (normalize to [-π, π])
        pitch_error = self._normalize_angle(current_pitch - desired_pitch)
        yaw_error = self._normalize_angle(current_yaw - desired_yaw)
        
        # PD control for gimbal commands
        gimbal_pitch = -self.Kp * pitch_error - self.Kd * current_pitch_rate
        gimbal_yaw = -self.Kp * yaw_error - self.Kd * current_yaw_rate
        
        # Clamp to gimbal limits
        gimbal_pitch = np.clip(gimbal_pitch, -self.gimbal_max, self.gimbal_max)
        gimbal_yaw = np.clip(gimbal_yaw, -self.gimbal_max, self.gimbal_max)
        
        return gimbal_pitch, gimbal_yaw
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-π, π]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class CascadedRocketController:
    """
    Complete cascaded trajectory tracking controller.
    
    Implements three nested control loops:
    1. Guidance: Position/velocity error → desired acceleration
    2. Attitude: Desired acceleration → desired orientation + throttle
    3. Stabilization: Orientation error → gimbal commands
    """
    
    def __init__(self, specs=None):
        """
        Initialize cascaded controller.
        
        Args:
            specs: RocketSpecs object with physical parameters
        """
        self.specs = specs or RocketSpecs()
        
        # Initialize sub-controllers
        self.guidance = GuidanceController(
            Kp=np.array([0.005, 0.005, 0.1]),    # Much stronger position gains
            Kd=np.array([0.1, 0.1, 0.2])         # Much stronger velocity damping
        )
        
        self.attitude = AttitudeController(
            max_thrust=self.specs.max_thrust,
            rocket_mass=self.specs.mass
        )
        
        self.stabilization = StabilizationController(
            Kp=1.0,   # Stronger attitude response
            Kd=0.3,   # Better damping
            gimbal_max=self.specs.gimbal_max_angle
        )
    
    def compute_control(self, rocket_state, desired_state, dt):
        """
        Compute control inputs (thrust, gimbal_pitch, gimbal_yaw) using cascaded loops.
        
        Args:
            rocket_state: Current rocket state
            desired_state: Desired trajectory state
            dt: Time step (seconds)
        
        Returns:
            (thrust, gimbal_pitch, gimbal_yaw)
        """
        # Extract current state
        actual_pos = rocket_state.position
        actual_vel = rocket_state.velocity
        pitch = rocket_state.orientation[1]
        yaw = rocket_state.orientation[2]
        pitch_rate = rocket_state.angular_velocity[1]
        yaw_rate = rocket_state.angular_velocity[2]
        
        # Extract desired state
        desired_pos = desired_state['position']
        desired_vel = desired_state['velocity']
        
        # === OUTER LOOP: Guidance ===
        desired_accel = self.guidance.compute_desired_acceleration(
            actual_pos, actual_vel, desired_pos, desired_vel
        )
        
        # === MIDDLE LOOP: Attitude ===
        throttle, desired_pitch, desired_yaw = self.attitude.compute_desired_attitude(
            desired_accel
        )

        # COAST LOGIC: if we're above the target altitude and still climbing, cut thrust
        altitude_error = actual_pos[2] - desired_pos[2]
        if altitude_error > 0.0 and actual_vel[2] > 0.0:
            throttle = 0.0
            desired_pitch = pitch
            desired_yaw = yaw
        
        # === INNER LOOP: Stabilization ===
        if throttle <= 0.0:
            gimbal_pitch, gimbal_yaw = 0.0, 0.0
        else:
            gimbal_pitch, gimbal_yaw = self.stabilization.compute_gimbal_angles(
                pitch, yaw, pitch_rate, yaw_rate,
                desired_pitch, desired_yaw
            )
        
        return throttle, gimbal_pitch, gimbal_yaw
    
    def reset(self):
        """Reset controller state if needed."""
        pass
