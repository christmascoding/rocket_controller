"""
Trajectory and Path Following System
Defines target paths and computes desired states for the rocket.
"""

import numpy as np
from src.config import PathParams

class TrajectoryPath:
    """Base class for trajectory paths."""
    
    def __init__(self):
        """Initialize trajectory."""
        self.total_time = 0.0
    
    def get_desired_state(self, time):
        """
        Get desired state at given time.
        
        Args:
            time: Current time (seconds)
        
        Returns:
            Dictionary with desired position, velocity, orientation
        """
        raise NotImplementedError


class VerticalLaunchPath(TrajectoryPath):
    """
    Vertical launch trajectory to 40km altitude.
    
    This path assumes the rocket accelerates to reach 40km altitude
    with zero velocity at apogee.
    """
    
    def __init__(self, target_altitude=40_000.0, gravity=9.81):
        """
        Initialize vertical launch path.
        
        Args:
            target_altitude: Target altitude in meters (default 40km)
            gravity: Gravitational acceleration (m/s^2)
        """
        super().__init__()
        self.target_altitude = target_altitude
        self.gravity = gravity
        
        # Compute required initial velocity (ballistic trajectory)
        # v^2 = 2 * g * h
        self.required_velocity = np.sqrt(2 * self.gravity * self.target_altitude)
        
        # Time to reach apogee from launch
        # v = v0 - g*t, at apogee v=0
        # t = v0 / g
        self.time_to_apogee = self.required_velocity / self.gravity
        
        # Total time to reach apogee
        self.total_time = self.time_to_apogee
    
    def get_desired_state(self, time):
        """
        Get desired state for vertical launch.
        
        During ascent: maintain vertical orientation, zero pitch/roll.
        
        Args:
            time: Current time (seconds)
        
        Returns:
            Dictionary with desired state
        """
        # Vertical path means position along z-axis only
        # Position increases linearly with velocity (kinematic equation)
        # z = v0*t - 0.5*g*t^2 (ballistic trajectory)
        z_desired = self.required_velocity * time - 0.5 * self.gravity * time**2
        
        # Velocity decreases due to gravity
        vz_desired = self.required_velocity - self.gravity * time
        
        desired_state = {
            'position': np.array([0.0, 0.0, z_desired]),
            'velocity': np.array([0.0, 0.0, vz_desired]),
            'orientation': np.array([0.0, 0.0, 0.0]),  # Vertical, no rotation
            'angular_velocity': np.array([0.0, 0.0, 0.0])
        }
        
        return desired_state


class SpiralPath(TrajectoryPath):
    """
    Spiral trajectory for ascent.
    The rocket spirals while ascending.
    """
    
    def __init__(self, target_altitude=40_000.0, num_spirals=5, gravity=9.81):
        """
        Initialize spiral path.
        
        Args:
            target_altitude: Target altitude in meters
            num_spirals: Number of complete spirals
            gravity: Gravitational acceleration (m/s^2)
        """
        super().__init__()
        self.target_altitude = target_altitude
        self.num_spirals = num_spirals
        self.gravity = gravity
        
        # Required velocity for ballistic trajectory
        self.required_velocity = np.sqrt(2 * self.gravity * self.target_altitude)
        self.time_to_apogee = self.required_velocity / self.gravity
        self.total_time = self.time_to_apogee
        
        # Spiral parameters
        self.horizontal_radius = 500.0  # 500m maximum horizontal displacement
    
    def get_desired_state(self, time):
        """
        Get desired state for spiral trajectory.
        
        Args:
            time: Current time (seconds)
        
        Returns:
            Dictionary with desired state
        """
        # Normalize time
        t_norm = np.clip(time / self.total_time, 0.0, 1.0)
        
        # Spiral angle (increases with time)
        angle = 2 * np.pi * self.num_spirals * t_norm
        
        # Altitude increases linearly
        z = self.target_altitude * t_norm
        
        # Horizontal position spirals
        radius = self.horizontal_radius * t_norm
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        
        # Vertical velocity decreases (ballistic)
        vz = self.required_velocity - self.gravity * time
        
        # Horizontal velocity components for spiral
        dr_dt = self.horizontal_radius / self.total_time
        dangle_dt = (2 * np.pi * self.num_spirals) / self.total_time
        
        vx = dr_dt * np.cos(angle) - radius * dangle_dt * np.sin(angle)
        vy = dr_dt * np.sin(angle) + radius * dangle_dt * np.cos(angle)
        
        # Desired orientation (keep pointing along velocity direction)
        v_horizontal = np.sqrt(vx**2 + vy**2)
        if v_horizontal > 0.1:
            pitch = np.arctan2(vz, v_horizontal)
            yaw = np.arctan2(vy, vx)
        else:
            pitch = np.pi / 2
            yaw = 0.0
        
        desired_state = {
            'position': np.array([x, y, z]),
            'velocity': np.array([vx, vy, vz]),
            'orientation': np.array([0.0, pitch, yaw]),  # roll, pitch, yaw
            'angular_velocity': np.array([0.0, 0.0, 0.0])
        }
        
        return desired_state


class ParabolicPath(TrajectoryPath):
    """
    Parabolic trajectory path.
    The rocket follows a parabolic arc.
    """
    
    def __init__(self, target_altitude=10_000.0, horizontal_range=5_000.0, gravity=9.81):
        """
        Initialize parabolic path.
        
        Args:
            target_altitude: Target altitude at apogee
            horizontal_range: Horizontal distance traveled
            gravity: Gravitational acceleration (m/s^2)
        """
        super().__init__()
        self.target_altitude = target_altitude
        self.horizontal_range = horizontal_range
        self.gravity = gravity
        
        # Compute required initial velocity components
        # For parabolic trajectory: h = (v0*sin(theta))^2 / (2*g)
        # range = (v0^2 * sin(2*theta)) / g
        # Solve for v0 and theta
        
        # Use angle of 45 degrees for maximum range
        self.launch_angle = np.radians(45.0)
        v0_y = np.sqrt(2 * self.gravity * self.target_altitude / (np.sin(self.launch_angle)**2))
        v0_x = v0_y * np.cos(self.launch_angle)
        
        self.v0 = np.array([v0_x, 0.0, v0_y])
        self.time_to_apogee = v0_y / self.gravity
        self.total_time = 2 * self.time_to_apogee
    
    def get_desired_state(self, time):
        """
        Get desired state for parabolic trajectory.
        
        Args:
            time: Current time (seconds)
        
        Returns:
            Dictionary with desired state
        """
        # Parabolic motion with gravity
        x = self.v0[0] * time
        z = self.v0[2] * time - 0.5 * self.gravity * time**2
        y = 0.0
        
        vx = self.v0[0]
        vz = self.v0[2] - self.gravity * time
        vy = 0.0
        
        # Orientation along velocity
        v_magnitude = np.linalg.norm([vx, vy, vz])
        if v_magnitude > 0.1:
            pitch = np.arctan2(vz, np.sqrt(vx**2 + vy**2))
            yaw = np.arctan2(vy, vx)
        else:
            pitch = 0.0
            yaw = 0.0
        
        desired_state = {
            'position': np.array([x, y, z]),
            'velocity': np.array([vx, vy, vz]),
            'orientation': np.array([0.0, pitch, yaw]),
            'angular_velocity': np.array([0.0, 0.0, 0.0])
        }
        
        return desired_state


class CurvedAscentPath(TrajectoryPath):
    """
    Curved ascent trajectory - proper ballistic arc.
    
    Rocket follows a ballistic parabola from origin to peak at (40km, 60km altitude).
    Then continues horizontally at 60km.
    """
    
    def __init__(self, peak_altitude=60_000.0, horizontal_distance=40_000.0, gravity=9.81):
        """
        Initialize curved ascent path using ballistic equations.
        
        Args:
            peak_altitude: Peak altitude (60km)
            horizontal_distance: Horizontal distance to peak (40km)
            gravity: Gravitational acceleration (9.81 m/s^2)
        """
        super().__init__()
        self.peak_altitude = peak_altitude
        self.horizontal_distance = horizontal_distance
        self.gravity = gravity
        
        # Phase 1: Ballistic arc to peak (40km, 60km) where vz=0
        # Using ballistic equations: z = v0_z*t - 0.5*g*t^2
        # At peak (vz=0): t_peak = v0_z / g
        # Peak altitude: z_peak = v0_z^2 / (2*g)
        
        # Solve for required initial vertical velocity
        self.v0_z = np.sqrt(2 * self.gravity * self.peak_altitude)
        self.t_peak = self.v0_z / self.gravity
        
        # Horizontal velocity to cover 40km by peak time
        self.v0_x = self.horizontal_distance / self.t_peak
        
        self.phase1_duration = self.t_peak
        
        # Phase 2: Horizontal cruise at peak altitude
        self.phase2_duration = 30.0
        self.total_time = self.phase1_duration + self.phase2_duration
    
    def get_desired_state(self, time):
        """
        Get desired state along curved trajectory.
        
        Args:
            time: Current time (seconds)
        
        Returns:
            Dictionary with desired state
        """
        # Phase 1: Ballistic ascent to (40km, 60km)
        if time <= self.phase1_duration:
            t = time
            
            # Ballistic equations
            x = self.v0_x * t
            z = self.v0_z * t - 0.5 * self.gravity * t**2
            y = 0.0
            
            # Velocity
            vx = self.v0_x
            vz = self.v0_z - self.gravity * t
            vy = 0.0
            
            # Orientation along velocity direction
            v_magnitude = np.sqrt(vx**2 + vz**2)
            if v_magnitude > 0.1:
                pitch = np.arctan2(vz, vx)
            else:
                pitch = 0.0
            yaw = 0.0
        
        # Phase 2: Horizontal cruise at peak altitude
        else:
            t_phase2 = time - self.phase1_duration
            
            # Continue horizontally from peak
            x = self.horizontal_distance + self.v0_x * t_phase2
            z = self.peak_altitude
            y = 0.0
            
            vx = self.v0_x
            vz = 0.0
            vy = 0.0
            
            pitch = 0.0
            yaw = 0.0
        
        desired_state = {
            'position': np.array([x, y, z]),
            'velocity': np.array([vx, vy, vz]),
            'orientation': np.array([0.0, pitch, yaw]),
            'angular_velocity': np.array([0.0, 0.0, 0.0])
        }
        
        return desired_state
