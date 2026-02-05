import numpy as np
import cvxpy as cp
from scipy.spatial.transform import Rotation as R
from config import Config

class MPCController:
    def __init__(self):
        self.dt = Config.dt
        self.N = Config.N_horizon
        
        self.Q = np.diag(Config.Q_diag)
        self.R = np.diag(Config.R_diag)

        # Build nominal A, B matrices (will be updated per step around ref trajectory)
        self.A, self.B = self._linearize_climbing()
        
        # Integral action for altitude error
        self.z_error_integral = 0.0
        self.ki_z = 0.05  # Integral gain for altitude
        
    def _linearize_climbing(self, ref_state=None):
        """
        Linearize around a climbing trajectory instead of hover.
        This improves accuracy when the rocket is climbing at significant velocity.
        
        State: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
               0  1  2   3   4   5    6      7    8   9  10  11
        """
        m = Config.mass
        g = Config.g
        drag_coeff = 0.1  # From rocket_model.py
        
        Ac = np.zeros((12, 12))
        
        # Kinematics: position derivatives = velocity
        Ac[0:3, 3:6] = np.eye(3)
        
        # Attitude kinematics: euler dot = ang_vel (small angle)
        Ac[6:9, 9:12] = np.eye(3)
        
        # Gravity coupling: tilt affects horizontal acceleration
        Ac[3, 7] = -g  # pitch positive -> -g acceleration in x
        Ac[4, 6] = g   # roll positive -> +g acceleration in y
        
        # Dynamics: velocity derivatives
        # Vertical acceleration from thrust (nominal at 1/m)
        Bc = np.zeros((12, 3))
        Bc[5, 0] = 1.0 / m  # thrust -> vz acceleration
        
        # Add damping from drag (linear approximation around operating velocity)
        # F_drag ≈ -0.1 * v_nominal * v (velocity dependent damping)
        if ref_state is not None:
            v_nom = np.linalg.norm(ref_state[3:6])
        else:
            v_nom = 1.0  # Nominal climbing at ~1 m/s
        
        # Drag acts as damping in body frame, approximated in inertial frame
        drag_damping = drag_coeff * v_nom
        Ac[3, 3] -= drag_damping / m  # Damping on vx
        Ac[4, 4] -= drag_damping / m  # Damping on vy
        Ac[5, 5] -= drag_damping / m  # Damping on vz
        
        # Gimbal control effects: gimbal angles create torques
        # which eventually feed back to attitude and acceleration
        lever = Config.L_com_to_gimbal
        F_ref = m * g + m * 2.0  # Reference thrust around 3g (climbing + margin)
        torque_per_rad = F_ref * lever
        
        # Gimbal Y (pitch): creates roll torque -> affects y acceleration
        Ac[4, 7] = torque_per_rad / Config.J[1, 1]  # Pitch rate -> roll acceleration
        Bc[10, 1] = torque_per_rad / Config.J[1, 1]  # Gimbal Y -> pitch rate
        
        # Gimbal Z (roll): creates pitch torque -> affects x acceleration  
        Ac[3, 6] = torque_per_rad / Config.J[2, 2]  # Roll rate -> pitch acceleration
        Bc[11, 2] = torque_per_rad / Config.J[2, 2]  # Gimbal Z -> roll rate
        
        # Discretize using forward Euler
        Ad = np.eye(12) + Ac * self.dt
        Bd = Bc * self.dt
        
        return Ad, Bd

    def compute(self, full_state, target_state):
        pos = full_state[0:3]
        vel = full_state[3:6]
        quat = full_state[6:10]
        w = full_state[10:13]
        
        # Convert Quat to Euler for MPC
        r = R.from_quat(quat)
        euler = r.as_euler('xyz')
        
        x0 = np.concatenate([pos, vel, euler, w])
        
        # Update linearization around current reference trajectory
        self.A, self.B = self._linearize_climbing(target_state)
        
        m = Config.mass
        g = Config.g
        
        # Extract target state components
        target_pos = target_state[0:3]
        target_vel = target_state[3:6]
        target_vz = target_vel[2]
        
        # Improved feedforward: compute thrust needed to follow target vertical trajectory
        # From vertical dynamics: vz_dot = thrust/m - g
        # To achieve target_vz and match reference acceleration:
        z_error = target_pos[2] - pos[2]
        vz_error = target_vel[2] - vel[2]
        
        # PD control on vertical velocity (with integral of position error)
        # When vz_error is large (initial transient), use aggressive derivative gain
        kp_z = 80.0   # Position error gain (increased)
        kd_z = 40.0   # Velocity error gain (increased for faster response)
        
        # Base thrust = m*g (hover) + m*target_vz_dot (feed-forward the target vertical acceleration)
        # Plus feedback on velocity and position errors
        # Assuming constant climb rate, target_vz_dot ≈ 0 for steady spiral
        thrust_ff = m * g + m * kd_z * vz_error + kp_z * z_error
        thrust_ff = np.clip(thrust_ff, Config.min_thrust, Config.max_thrust)
        
        x = cp.Variable((12, self.N + 1))
        u = cp.Variable((3, self.N))
        
        cost = 0
        constraints = [x[:, 0] == x0]
        
        # Reference trajectory
        xref = np.zeros(12)
        xref[0:3] = target_state[0:3]  
        xref[3:6] = target_state[3:6]  
        
        for k in range(self.N):
            cost += cp.quad_form(x[:, k] - xref, self.Q)
            # Penalize deviation from calculated feedforward thrust and Zero Gimbal
            u_dev = u[:, k] - np.array([thrust_ff, 0, 0])
            cost += cp.quad_form(u_dev, self.R)
            
            constraints += [x[:, k+1] == self.A @ x[:, k] + self.B @ u[:, k]]
            
            # Actuator Limits
            constraints += [u[0, k] >= Config.min_thrust, u[0, k] <= Config.max_thrust]
            constraints += [cp.abs(u[1, k]) <= Config.max_gimbal]
            constraints += [cp.abs(u[2, k]) <= Config.max_gimbal]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            return u[:, 0].value, x.value
        else:
            return np.array([thrust_ff, 0, 0]), np.zeros((12, self.N + 1))