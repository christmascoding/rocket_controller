"""
6-DOF Rocket Physics Model
Handles translational and rotational dynamics of the rocket.
"""

import numpy as np
from src.config import RocketSpecs, SimulationParams

class RocketState:
    """Encapsulates the complete state of the rocket."""
    
    def __init__(self, position=None, velocity=None, orientation=None, angular_velocity=None):
        """
        Initialize rocket state.
        
        Args:
            position: [x, y, z] position in meters
            velocity: [vx, vy, vz] velocity in m/s
            orientation: [alpha1, alpha2, alpha3] Euler angles in radians
            angular_velocity: [p, q, r] angular rates in rad/s
        """
        self.position = np.array(position) if position is not None else np.zeros(3)
        self.velocity = np.array(velocity) if velocity is not None else np.zeros(3)
        self.orientation = np.array(orientation) if orientation is not None else np.zeros(3)  # Euler angles
        self.angular_velocity = np.array(angular_velocity) if angular_velocity is not None else np.zeros(3)
    
    def copy(self):
        """Return a deep copy of the state."""
        return RocketState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            orientation=self.orientation.copy(),
            angular_velocity=self.angular_velocity.copy()
        )


class Rocket:
    """6-DOF Rocket physics model."""
    
    def __init__(self, specs=None):
        """
        Initialize rocket with specifications.
        
        Args:
            specs: RocketSpecs object with physical parameters
        """
        self.specs = specs or RocketSpecs()
        self.state = RocketState()
        
        # Control inputs
        self.thrust = 0.0  # Normalized thrust (0-1)
        self.gimbal_pitch = 0.0  # Gimbal angle in pitch (radians)
        self.gimbal_yaw = 0.0  # Gimbal angle in yaw (radians)
    
    def set_control_inputs(self, thrust, gimbal_pitch, gimbal_yaw):
        """
        Set control inputs for the rocket.
        
        Args:
            thrust: Normalized thrust (0.0 to 1.0)
            gimbal_pitch: Gimbal pitch angle (radians, clamped)
            gimbal_yaw: Gimbal yaw angle (radians, clamped)
        """
        self.thrust = np.clip(thrust, 0.0, 1.0)
        self.gimbal_pitch = np.clip(gimbal_pitch, -self.specs.gimbal_max_angle, self.specs.gimbal_max_angle)
        self.gimbal_yaw = np.clip(gimbal_yaw, -self.specs.gimbal_max_angle, self.specs.gimbal_max_angle)
    
    def _compute_rotation_matrix(self, alpha1, alpha2, alpha3):
        """
        Compute rotation matrix from body-fixed to inertial frame.
        Using ZYX Euler angles (yaw-pitch-roll).
        
        alpha1: roll (rotation about x-axis)
        alpha2: pitch (rotation about y-axis)
        alpha3: yaw (rotation about z-axis)
        """
        # Rotation matrices
        c1, s1 = np.cos(alpha1), np.sin(alpha1)
        c2, s2 = np.cos(alpha2), np.sin(alpha2)
        c3, s3 = np.cos(alpha3), np.sin(alpha3)
        
        # ZYX rotation matrix (yaw-pitch-roll)
        R = np.array([
            [c3*c2, c3*s2*s1 - s3*c1, c3*s2*c1 + s3*s1],
            [s3*c2, s3*s2*s1 + c3*c1, s3*s2*c1 - c3*s1],
            [-s2,   c2*s1,            c2*c1]
        ])
        
        return R
    
    def _compute_thrust_vector(self):
        """
        Compute thrust vector in body frame due to gimbal angles.
        
        In the body frame, the rocket points upward along +Z axis.
        The engine produces thrust in the +Z direction.
        Gimbal angles deflect the engine.
        
        Positive gimbal angles cause deflection in positive directions,
        which creates appropriate torques about the body axes.
        
        Returns:
            Thrust vector in body frame [Fx, Fy, Fz]
        """
        F_magnitude = self.thrust * self.specs.max_thrust
        
        # Gimbal deflection: invert the sign so positive gimbal angle
        # creates thrust in positive direction (which creates correct torque)
        # Positive pitch gimbal deflects thrust to -Y (which creates positive pitch torque)
        # Positive yaw gimbal deflects thrust to -X (which creates positive yaw torque)
        Fx = -F_magnitude * np.sin(self.gimbal_yaw)
        Fy = -F_magnitude * np.sin(self.gimbal_pitch)
        Fz = F_magnitude * np.cos(self.gimbal_pitch) * np.cos(self.gimbal_yaw)  # Still positive for upward thrust
        
        return np.array([Fx, Fy, Fz])
    
    def _compute_aerodynamic_forces(self):
        """
        Compute aerodynamic drag in body frame.
        Simplified model: drag proportional to velocity squared.
        
        Returns:
            Drag force vector [Fx, Fy, Fz]
        """
        # Convert velocity to body frame
        R = self._compute_rotation_matrix(
            self.state.orientation[0],
            self.state.orientation[1],
            self.state.orientation[2]
        )
        
        # Body frame velocity
        v_body = R.T @ self.state.velocity
        
        # Dynamic pressure
        v_magnitude = np.linalg.norm(self.state.velocity)
        if v_magnitude < 0.1:
            return np.zeros(3)
        
        q = 0.5 * SimulationParams.air_density * v_magnitude**2
        
        # Drag acts opposite to velocity direction
        # Simplified: drag in the direction of motion
        drag_magnitude = q * self.specs.drag_coefficient * self.specs.reference_area
        
        # Drag vector in inertial frame (opposite to velocity)
        if v_magnitude > 0:
            drag_inertial = -drag_magnitude * (self.state.velocity / v_magnitude)
        else:
            drag_inertial = np.zeros(3)
        
        return drag_inertial
    
    def _compute_lift_force(self):
        """
        Compute lift force from ailerons/fins.
        
        Ailerons generate lift perpendicular to the rocket's body axis.
        This creates SIDEWAYS forces and motion, not rotation.
        
        The lift acts in the plane perpendicular to the rocket's pointing direction,
        helping it "glide" sideways based on its orientation and velocity.
        
        Returns:
            Lift force vector in inertial frame [Fx, Fy, Fz]
        """
        v_magnitude = np.linalg.norm(self.state.velocity)
        
        # Need sufficient velocity for lift
        if v_magnitude < 1.0:
            return np.zeros(3)
        
        # Rocket body axis (points upward in body frame along +Z)
        R = self._compute_rotation_matrix(
            self.state.orientation[0],
            self.state.orientation[1],
            self.state.orientation[2]
        )
        rocket_axis_body = np.array([0.0, 0.0, 1.0])
        rocket_axis_inertial = R @ rocket_axis_body
        
        # Velocity direction
        velocity_dir = self.state.velocity / v_magnitude
        
        # Angle of attack
        cos_attack = np.dot(rocket_axis_inertial, velocity_dir)
        cos_attack = np.clip(cos_attack, -1.0, 1.0)
        
        # If nearly aligned, minimal lift
        if abs(cos_attack) > 0.98:
            return np.zeros(3)
        
        # Perpendicular component: velocity's component perpendicular to rocket axis
        # This is the "crosswind" direction relative to the rocket
        perp_velocity = velocity_dir - cos_attack * rocket_axis_inertial
        perp_velocity_mag = np.linalg.norm(perp_velocity)
        
        if perp_velocity_mag < 0.01:
            return np.zeros(3)
        
        # Normalize perpendicular velocity direction
        perp_velocity_dir = perp_velocity / perp_velocity_mag
        
        # Dynamic pressure
        q = 0.5 * SimulationParams.air_density * v_magnitude**2
        
        # Lift magnitude based on angle of attack
        sin_attack = np.sqrt(1.0 - cos_attack**2)
        
        # Aileron lift: generates force perpendicular to rocket axis
        # in the direction of the relative crosswind
        lift_magnitude = q * self.specs.aileron_area * self.specs.lift_coefficient * sin_attack
        
        # Lift acts in the perpendicular velocity direction
        # (sideways to how the rocket is pointed)
        lift_force = lift_magnitude * perp_velocity_dir
        
        return lift_force
    
    def _compute_gravity_force(self):
        """
        Compute gravitational force in inertial frame.
        
        Returns:
            Gravity force vector [Fx, Fy, Fz] (points downward in z)
        """
        return np.array([0.0, 0.0, -self.specs.mass * SimulationParams.gravity])
    
    def _compute_torques(self):
        """
        Compute torques due to gimbal deflection.
        Gimbal offset creates moment arm from CG to engine.
        
        Returns:
            Torque vector in body frame [tau_x, tau_y, tau_z]
        """
        # Thrust vector in body frame
        thrust_vec = self._compute_thrust_vector()
        
        # Engine position relative to CG (in body frame)
        # Engine is at -engine_offset_from_cg along z-axis in body frame
        engine_pos_body = np.array([0.0, 0.0, -self.specs.engine_offset_from_cg])
        
        # Torque = r × F
        torque = np.cross(engine_pos_body, thrust_vec)
        
        return torque
    
    def update(self, dt):
        """
        Update rocket state for one time step using 4th-order Runge-Kutta.
        
        Args:
            dt: Time step (seconds)
        """
        def derivatives(state):
            """Compute state derivatives."""
            # Rotation matrix from body to inertial
            R = self._compute_rotation_matrix(
                state.orientation[0],
                state.orientation[1],
                state.orientation[2]
            )
            
            # Forces in inertial frame
            thrust_body = self._compute_thrust_vector()
            thrust_inertial = R @ thrust_body
            gravity = self._compute_gravity_force()
            drag = self._compute_aerodynamic_forces()
            lift = self._compute_lift_force()
            
            total_force = thrust_inertial + gravity + drag + lift
            
            # Linear acceleration
            acceleration = total_force / self.specs.mass
            
            # Torques in body frame
            torques = self._compute_torques()
            
            # Angular acceleration (using simplified Euler dynamics)
            # I * alpha = tau - omega × (I * omega)
            I = np.array([
                [self.specs.Ixx, 0, 0],
                [0, self.specs.Iyy, 0],
                [0, 0, self.specs.Izz]
            ])
            
            I_omega = I @ state.angular_velocity
            gyro_term = np.cross(state.angular_velocity, I_omega)
            angular_acceleration = np.linalg.solve(I, torques - gyro_term)
            
            # Euler angle rates from angular velocity (ZYX convention)
            alpha1, alpha2, alpha3 = state.orientation
            c1, s1 = np.cos(alpha1), np.sin(alpha1)
            c2, s2 = np.cos(alpha2), np.sin(alpha2)
            
            if abs(c2) < 1e-6:  # Singularity near pitch = ±90°
                c2 = 1e-6
            
            # Transform angular velocity to Euler angle rates
            dalpha1_dt = state.angular_velocity[0] + s1 * np.tan(alpha2) * state.angular_velocity[1] + c1 * np.tan(alpha2) * state.angular_velocity[2]
            dalpha2_dt = c1 * state.angular_velocity[1] - s1 * state.angular_velocity[2]
            dalpha3_dt = s1 / c2 * state.angular_velocity[1] + c1 / c2 * state.angular_velocity[2]
            
            orientation_rates = np.array([dalpha1_dt, dalpha2_dt, dalpha3_dt])
            
            return {
                'position_dot': state.velocity.copy(),
                'velocity_dot': acceleration.copy(),
                'orientation_dot': orientation_rates.copy(),
                'angular_velocity_dot': angular_acceleration.copy()
            }
        
        # RK4 integration
        k1 = derivatives(self.state)
        
        state_k2 = self.state.copy()
        state_k2.position += 0.5 * dt * k1['position_dot']
        state_k2.velocity += 0.5 * dt * k1['velocity_dot']
        state_k2.orientation += 0.5 * dt * k1['orientation_dot']
        state_k2.angular_velocity += 0.5 * dt * k1['angular_velocity_dot']
        
        k2 = derivatives(state_k2)
        
        state_k3 = self.state.copy()
        state_k3.position += 0.5 * dt * k2['position_dot']
        state_k3.velocity += 0.5 * dt * k2['velocity_dot']
        state_k3.orientation += 0.5 * dt * k2['orientation_dot']
        state_k3.angular_velocity += 0.5 * dt * k2['angular_velocity_dot']
        
        k3 = derivatives(state_k3)
        
        state_k4 = self.state.copy()
        state_k4.position += dt * k3['position_dot']
        state_k4.velocity += dt * k3['velocity_dot']
        state_k4.orientation += dt * k3['orientation_dot']
        state_k4.angular_velocity += dt * k3['angular_velocity_dot']
        
        k4 = derivatives(state_k4)
        
        # RK4 update
        self.state.position += (dt / 6.0) * (k1['position_dot'] + 2*k2['position_dot'] + 2*k3['position_dot'] + k4['position_dot'])
        self.state.velocity += (dt / 6.0) * (k1['velocity_dot'] + 2*k2['velocity_dot'] + 2*k3['velocity_dot'] + k4['velocity_dot'])
        self.state.orientation += (dt / 6.0) * (k1['orientation_dot'] + 2*k2['orientation_dot'] + 2*k3['orientation_dot'] + k4['orientation_dot'])
        self.state.angular_velocity += (dt / 6.0) * (k1['angular_velocity_dot'] + 2*k2['angular_velocity_dot'] + 2*k3['angular_velocity_dot'] + k4['angular_velocity_dot'])
    
    def get_cg_position(self):
        """Get center of gravity position."""
        return self.state.position.copy()
    
    def get_thrust_vector_inertial(self):
        """Get thrust vector in inertial frame."""
        R = self._compute_rotation_matrix(
            self.state.orientation[0],
            self.state.orientation[1],
            self.state.orientation[2]
        )
        thrust_body = self._compute_thrust_vector()
        return R @ thrust_body
