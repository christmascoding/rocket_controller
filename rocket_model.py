import numpy as np
from scipy.integrate import odeint
from scipy.spatial.transform import Rotation as R
from config import Config  # Import the shared config

class Actuator:
    def __init__(self, tau, limits):
        self.tau = tau
        self.min_val, self.max_val = limits
        self.state = 0.0
    
    def update(self, cmd, dt):
        cmd_clamped = np.clip(cmd, self.min_val, self.max_val)
        self.state += (cmd_clamped - self.state) * (dt / self.tau)
        return self.state

class RocketModel:
    def __init__(self):
        # State: [x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]
        self.state = np.zeros(13)
        self.state[2] = 0.0
        self.state[9] = 1.0  # Identity Quaternion
        
        self.eng_thrust = Actuator(Config.tau_thrust, (Config.min_thrust, Config.max_thrust))
        self.eng_gimbal_y = Actuator(Config.tau_gimbal, (-Config.max_gimbal, Config.max_gimbal))
        self.eng_gimbal_z = Actuator(Config.tau_gimbal, (-Config.max_gimbal, Config.max_gimbal))

    def dynamics(self, state, t, u_lagged):
        r = state[0:3]
        v = state[3:6]
        q = state[6:10]
        w = state[10:13]
        
        F_th, delta_y, delta_z = u_lagged
        
        # Forces (Body Frame)
        F_body = np.array([
            F_th * np.sin(delta_y), 
            F_th * np.sin(delta_z), 
            F_th * np.cos(np.sqrt(delta_y**2 + delta_z**2))
        ])
        
        # Drag
        v_body = R.from_quat(q).inv().apply(v)
        F_drag = -0.1 * v_body * np.linalg.norm(v_body)
        F_tot_body = F_body + F_drag
        
        # Moments
        r_eng = np.array([0, 0, -Config.L/2])
        M_tot = np.cross(r_eng, F_body)
        
        # Newton-Euler
        F_inertial = R.from_quat(q).apply(F_tot_body)
        g_vec = np.array([0, 0, -Config.g])
        a_inertial = F_inertial/Config.mass + g_vec
        
        Iw = Config.J @ w
        w_dot = np.linalg.inv(Config.J) @ (M_tot - np.cross(w, Iw))
        
        Omega = np.array([
            [0,    w[2], -w[1], w[0]],
            [-w[2], 0,    w[0], w[1]],
            [w[1], -w[0], 0,    w[2]],
            [-w[0], -w[1], -w[2], 0]
        ])
        q_dot = 0.5 * Omega @ q
        
        return np.concatenate((v, a_inertial, q_dot, w_dot))

    def step(self, dt, u_cmd):
        f = self.eng_thrust.update(u_cmd[0], dt)
        gy = self.eng_gimbal_y.update(u_cmd[1], dt)
        gz = self.eng_gimbal_z.update(u_cmd[2], dt)
        
        u_lagged = [f, gy, gz]
        sol = odeint(self.dynamics, self.state, [0, dt], args=(u_lagged,))
        self.state = sol[-1]
        self.state[6:10] /= np.linalg.norm(self.state[6:10]) # Re-normalize quaternion
        return self.state