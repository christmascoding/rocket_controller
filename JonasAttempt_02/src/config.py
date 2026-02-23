"""
Configuration file for rocket physics and control simulation.
Contains all rocket specifications and simulation parameters.
"""

import numpy as np

# Rocket Physical Specifications
class RocketSpecs:
    # Dimensions
    diameter = 0.70  # 70cm in meters
    length = 5.825   # 5.825m
    
    # Mass and propulsion
    mass = 10_000.0  # 10 tonnes in kg
    max_thrust = 300_000.0  # 300kN in Newtons
    
    # Gimbal
    gimbal_max_angle = np.radians(15.0)  # 15 degrees in radians
    gimbal_point_to_cg = 2.8  # Center of mass is 2.8m above gimbal point
    
    # Engine parameters
    engine_offset_from_cg = 2.8  # Distance from CG to engine gimbal point (z-axis)
    
    # Aerodynamic parameters
    reference_area = np.pi * (diameter / 2) ** 2  # Cross-sectional area
    drag_coefficient = 0.25  # Typical for rocket
    
    # Aileron/fin parameters for gliding behavior
    aileron_area = 0.5  # m^2 - small fins for stability
    lift_coefficient = 0.5  # Lift coefficient (simplified, constant with angle)
    
    # Moments of inertia (approximate for thin cylindrical body)
    # Ixx, Iyy similar (perpendicular to long axis)
    # Izz is about the long axis (much smaller)
    radius = diameter / 2
    # Simplified moment of inertia calculations
    Ixx = (1/12) * mass * (3 * radius**2 + length**2)  # About x-axis
    Iyy = (1/12) * mass * (3 * radius**2 + length**2)  # About y-axis
    Izz = (1/2) * mass * radius**2                      # About z-axis

# Simulation Parameters
class SimulationParams:
    dt = 0.01  # Time step in seconds
    gravity = 9.81  # m/s^2
    air_density = 1.225  # kg/m^3 at sea level
    max_simulation_time = 300.0  # Maximum simulation time (5 minutes)
    
    # Damping for numerical stability
    angular_damping = 0.0  # Small damping for realistic rotation

# Gimbal and Control Limits
class ControlLimits:
    # Thrust
    min_thrust = 0.0  # 0% thrust
    max_thrust = 1.0  # 100% thrust
    
    # Gimbal angles
    max_gimbal_pitch = RocketSpecs.gimbal_max_angle
    max_gimbal_yaw = RocketSpecs.gimbal_max_angle

# Visualization Parameters
class VisualizationParams:
    # Thrust visualization line
    thrust_line_min_length = 1.0  # 1m at 0% thrust
    thrust_line_max_length = 10.0  # 10m at 100% thrust
    
    # Colors for thrust line
    thrust_min_color = [0, 1, 0]  # Green at 0% thrust
    thrust_max_color = [1, 0, 0]  # Red at 100% thrust
    
    # Plot update frequency
    plot_update_hz = 30  # Update plots at 30 Hz
    
    # Camera parameters
    rotation_speed = 2.0  # degrees per second for automatic rotation

# Initial Conditions
class InitialConditions:
    # Position (rocket starts at origin with CG at 0,0,0)
    position = np.array([0.0, 0.0, 0.0])  # [x, y, z]
    
    # Velocity - start with 5 m/s upward to help with liftoff
    velocity = np.array([0.0, 0.0, 5.0])  # [vx, vy, vz]
    
    # Orientation (Euler angles: roll, pitch, yaw)
    # Rocket initially vertical (pointing up in -z direction if we use NED)
    # Actually pointing up means pitch=0, roll=0, yaw=0 in standard coords
    orientation = np.array([0.0, 0.0, 0.0])  # [alpha1, alpha2, alpha3]
    
    # Angular velocity
    angular_velocity = np.array([0.0, 0.0, 0.0])  # [p, q, r]

# Path Following Parameters
class PathParams:
    vertical_target_altitude = 60_000.0  # 60km target altitude (increased from 40km)
    target_velocity_at_apogee = 0.0  # Velocity at the peak
