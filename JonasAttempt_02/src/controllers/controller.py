"""
Rocket Path Following Controller
Computes control inputs to follow desired trajectories.
"""

import numpy as np
from src.config import RocketSpecs, ControlLimits

class PIDController:
    """Simple PID controller for scalar feedback control."""
    
    def __init__(self, kp, ki, kd, output_min=-1.0, output_max=1.0):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral = 0.0
        self.last_error = 0.0
    
    def update(self, error, dt):
        """
        Update controller and compute output.
        
        Args:
            error: Current error (desired - actual)
            dt: Time step (seconds)
        
        Returns:
            Control output
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0
        
        self.last_error = error
        
        # Compute output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = np.clip(output, self.output_min, self.output_max)
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.last_error = 0.0


class RocketController:
    """
    Path following controller for the rocket.
    
    Uses attitude control to follow a desired trajectory.
    """
    
    def __init__(self, specs=None):
        """
        Initialize controller.
        
        Args:
            specs: RocketSpecs object
        """
        self.specs = specs or RocketSpecs()
        
        # Altitude controller (controls thrust)
        # Prevents overshooting target altitude by modulating thrust
        self.altitude_controller = PIDController(kp=0.02, ki=0.001, kd=0.01, output_min=0.0, output_max=1.0)
        
        # Pitch controller (controls pitch gimbal) - MUCH reduced gains to prevent oscillation
        self.pitch_controller = PIDController(kp=0.05, ki=0.01, kd=0.01, 
                                             output_min=-self.specs.gimbal_max_angle, 
                                             output_max=self.specs.gimbal_max_angle)
        
        # Yaw controller (controls yaw gimbal) - MUCH reduced gains to prevent oscillation
        self.yaw_controller = PIDController(kp=0.05, ki=0.01, kd=0.01,
                                           output_min=-self.specs.gimbal_max_angle,
                                           output_max=self.specs.gimbal_max_angle)
        
        # Attitude stabilization controllers
        self.roll_stabilizer = PIDController(kp=0.2, ki=0.01, kd=0.05)
        self.pitch_stabilizer = PIDController(kp=0.2, ki=0.01, kd=0.05)
        self.yaw_stabilizer = PIDController(kp=0.2, ki=0.01, kd=0.05)
    
    def compute_control(self, rocket_state, desired_state, dt):
        """
        Compute control inputs to follow desired trajectory.
        
        Uses proper trajectory tracking control:
        1. Compute tracking errors (position and velocity)
        2. Design desired acceleration based on errors
        3. Calculate gimbal angles needed to produce desired acceleration
        
        Args:
            rocket_state: Current rocket state (RocketState object)
            desired_state: Desired state (dictionary with position, velocity, orientation)
            dt: Time step (seconds)
        
        Returns:
            Tuple (thrust, gimbal_pitch, gimbal_yaw) - control inputs
        """
        # Extract current state
        current_pos = rocket_state.position
        current_vel = rocket_state.velocity
        current_orient = rocket_state.orientation
        
        # Extract desired state
        desired_pos = desired_state['position']
        desired_vel = desired_state['velocity']
        
        # ========== THRUST CONTROL ==========
        # Modulate thrust based on altitude error to prevent overshooting
        altitude_error = current_pos[2] - desired_pos[2]  # positive if too high
        
        # Use PID to modulate thrust based on altitude error
        thrust = self.altitude_controller.update(-altitude_error, dt)
        
        # Ensure minimum thrust during ascent, maximum at all times
        if desired_vel[2] > 100.0:  # Still ascending (upward velocity > 100 m/s)
            thrust = max(thrust, 0.5)  # Maintain at least 50% thrust during ascent
        else:
            thrust = max(thrust, 0.1)  # Minimum 10% thrust for stability
        
        # ========== TRAJECTORY TRACKING GIMBAL CONTROL ==========
        # Proper control theory approach:
        # 1. Compute tracking errors in position and velocity
        # 2. Design desired acceleration to drive these errors to zero
        # 3. Calculate gimbal angles needed to produce desired acceleration vector
        
        # Compute position and velocity errors
        pos_error = current_pos - desired_pos
        vel_error = current_vel - desired_vel
        
        # Gain coefficients for trajectory tracking
        Kp = np.array([0.001, 0.001, 0.01])  # Position gain (very low - can't fight gravity)
        Kv = np.array([0.05, 0.05, 0.05])    # Velocity gain (moderate)
        
        # Gravity acceleration (always acts downward)
        g = 9.81
        gravity = np.array([0.0, 0.0, -g])
        
        # Desired acceleration from trajectory tracking error
        # a_desired = gravity - Kp*pos_error - Kv*vel_error
        desired_accel = gravity - Kp * pos_error - Kv * vel_error
        
        # Current rocket attitude (pitch, yaw from Euler angles)
        pitch = current_orient[1]
        yaw = current_orient[2]
        
        # Physical parameters
        max_thrust = 300_000.0  # Newtons
        rocket_mass = 50_000.0  # kg
        thrust_magnitude = thrust * max_thrust
        
        # Gimbal control: compute angles to achieve desired acceleration
        if thrust_magnitude > 100.0:
            # Desired thrust direction = (desired_accel - gravity) * mass / thrust
            desired_thrust_accel = desired_accel - gravity
            desired_thrust_dir = desired_thrust_accel * rocket_mass / thrust_magnitude
            
            # Clamp to magnitude ≤ 1 (can't exceed available thrust)
            desired_dir_norm = np.linalg.norm(desired_thrust_dir)
            if desired_dir_norm > 1.0:
                desired_thrust_dir = desired_thrust_dir / desired_dir_norm
            
            # Convert desired direction to pitch/yaw angles
            # Thrust direction (after gimbal): [sin(pitch)*sin(yaw), sin(pitch)*cos(yaw), cos(pitch)]
            if np.abs(desired_thrust_dir[2]) < 0.99:
                desired_pitch = np.arctan2(
                    np.sqrt(desired_thrust_dir[0]**2 + desired_thrust_dir[1]**2),
                    desired_thrust_dir[2]
                )
                desired_yaw = np.arctan2(desired_thrust_dir[0], desired_thrust_dir[1])
            else:
                # Near vertical - avoid singularity
                desired_pitch = 0.0
                desired_yaw = yaw
            
            # Gimbal command: difference between desired and current angles
            gimbal_max = self.specs.gimbal_max_angle
            gimbal_pitch = np.clip(desired_pitch - pitch, -gimbal_max, gimbal_max)
            gimbal_yaw = np.clip(desired_yaw - yaw, -gimbal_max, gimbal_max)
        else:
            gimbal_pitch = 0.0
            gimbal_yaw = 0.0
        
        return thrust, gimbal_pitch, gimbal_yaw
    
    def reset(self):
        """Reset all controller states."""
        self.altitude_controller.reset()
        self.pitch_controller.reset()
        self.yaw_controller.reset()
        self.roll_stabilizer.reset()
        self.pitch_stabilizer.reset()
        self.yaw_stabilizer.reset()
