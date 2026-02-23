import numpy as np
from scipy.integrate import solve_ivp

from src import config
from src.controllers.cascaded import (
    outer_loop,
    middle_loop,
    inner_loop,
    aero_surface_loop,
    phase3_controller,
    pure_pursuit,
    phase2_flip_controller,
)
from src.models.trajectory import sample_trajectory
from src.utils import body_z_axis_in_world
from src.physics.dynamics import state_derivative


class Simulator:
    def __init__(self, rocket, traj_t, traj_pos, traj_vel):
        self.rocket = rocket
        self.traj_t = traj_t
        self.traj_pos = traj_pos
        self.traj_vel = traj_vel

        # State: [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
        self.state = np.zeros(12)
        self.state[2] = 0.0
        self.state[5] = 50.0

        self.history = {
            "t": [],
            "state": [],
            "control": [],
            "target": [],
            "phase": [],
        }

        self.phase = "phase1a"
        self.landing_target = np.array([0.0, 0.0])
        self.phase3_start_time = None
        self.prev_gimbal = (0.0, 0.0)

    def _should_ignite(self):
        z = self.state[2]
        vz = self.state[5]
        if z <= 0.0:
            return False

        amax = self.rocket.max_thrust / self.rocket.mass - config.G
        if amax <= 0.1:
            return False

        dstop = (vz ** 2) / (2.0 * amax)
        # Add safety margin: trigger earlier (dstop >= 0.85*Z instead of >= Z)
        trigger_altitude = z * config.PHASE3_IGNITION_SAFETY_MARGIN
        should_trigger = vz < 0.0 and dstop >= trigger_altitude
        
        # Telemetry: print d_stop vs Z during Phase 2
        if self.phase == "phase2" and vz < 0.0:
            print(f"Phase2: Z={z:.1f}m, Vz={vz:.1f}m/s, d_stop={dstop:.1f}m, trigger_alt={trigger_altitude:.1f}m")
        
        return should_trigger

    def _update_phase(self, t):
        prev_phase = self.phase
        if self.phase == "phase1a" and (
            t >= config.MECO_TIME_MAX
            or self.state[0] >= config.TRAJ_X_REF
            or self.state[2] >= config.MECO_ALT_MAX
        ):
            self.phase = "phase1b"
            print(f"--- TRANSITION TO PHASE 1b (MECO/COAST) at t={t:.1f}s, Z={self.state[2]:.0f}m ---")
        if self.phase == "phase1b" and self.state[5] <= 0.0:
            self.phase = "phase2"
            print("--- TRANSITION TO PHASE 2 (DESCENT) ---")
            # Instant flip and zero rates for clean descent
            self.state[7] = np.pi  # theta = 180 deg
            self.state[6] = 0.0    # phi = 0
            self.state[8] = 0.0    # psi = 0
            self.state[9:12] = 0.0 # zero all angular rates
        if self.phase == "phase2" and self._should_ignite():
            self.phase = "phase3"
            self.landing_target = self.state[0:2].copy()
            self.phase3_start_time = t
            print(f"\n{'='*70}")
            print(f"--- TRANSITION TO PHASE 3 (HOVERSLAM) ---")
            print(f"  Time: t={t:.1f}s")
            print(f"  Position: X={self.state[0]:.1f}m, Y={self.state[1]:.1f}m, Z={self.state[2]:.0f}m")
            print(f"  Velocity: Vx={self.state[3]:.1f}m/s, Vy={self.state[4]:.1f}m/s, Vz={self.state[5]:.1f}m/s")
            print(f"  Attitude: φ={np.rad2deg(self.state[6]):.1f}°, θ={np.rad2deg(self.state[7]):.1f}°, ψ={np.rad2deg(self.state[8]):.1f}°")
            print(f"  Rates: p={np.rad2deg(self.state[9]):.1f}°/s, q={np.rad2deg(self.state[10]):.1f}°/s, r={np.rad2deg(self.state[11]):.1f}°/s")
            print(f"{'='*70}\n")
            
            # CRITICAL: Complete state reset for clean LQR handover
            self.state[9:12] = np.zeros(3)  # Zero ALL angular rates [p, q, r]
            self.prev_gimbal = (0.0, 0.0)  # Zero gimbal history (no fin inheritance!)
        
        # Landing detection
        if self.phase == "phase3" and self.state[2] < 0.5 and self.state[5] > -1.0:
            self.phase = "landed"
            print(f"\n{'='*70}")
            print(f"*** LANDING SUCCESSFUL ***")
            print(f"  Time: t={t:.1f}s")
            print(f"  Final Position: X={self.state[0]:.1f}m, Y={self.state[1]:.1f}m, Z={self.state[2]:.2f}m")
            print(f"  Final Velocity: Vx={self.state[3]:.2f}m/s, Vy={self.state[4]:.2f}m/s, Vz={self.state[5]:.2f}m/s")
            print(f"  Landing Error: {np.linalg.norm(self.state[0:2] - self.landing_target):.1f}m from target")
            print(f"{'='*70}\n")

    def compute_control(self, t):
        self._update_phase(t)
        pos = self.state[0:3]
        vel = self.state[3:6]
        phi, theta, psi = self.state[6:9]
        omega = self.state[9:12]

        if self.phase == "phase1a":
            # Pure Pursuit guidance
            theta_des, carrot_pos, end_of_path = pure_pursuit(pos, self.traj_pos)
            
            if end_of_path:
                # Hold current attitude until apogee
                u_des = body_z_axis_in_world(phi, theta, psi)
            else:
                # Convert desired pitch to desired direction vector
                # theta_des = 0 means straight up (Z), positive theta_des means pitch toward +X
                u_des = np.array([np.sin(theta_des), 0.0, np.cos(theta_des)])
            
            throttle = 1.0
            delta_y, delta_z = inner_loop(
                phi,
                theta,
                psi,
                omega,
                u_des,
                throttle,
                self.rocket.max_thrust,
                self.rocket.cg_from_gimbal,
                gimbal_limit=config.LAUNCH_GIMBAL_MAX if t < config.LAUNCH_T_LIMIT else None,
            )
            aero_torque = np.zeros(3)
            grid_fins = 0.0
            pos_des = carrot_pos
        elif self.phase == "phase1b":
            pos_des, vel_des = sample_trajectory(t, self.traj_t, self.traj_pos, self.traj_vel)
            f_des = outer_loop(pos, vel, pos_des, vel_des, self.rocket.mass)
            u_des, _ = middle_loop(
                f_des, self.rocket.max_thrust, tilt_gain=config.MIDDLE_TILT_GAIN
            )
            throttle = 0.0
            delta_y, delta_z = inner_loop(
                phi,
                theta,
                psi,
                omega,
                u_des,
                throttle,
                self.rocket.max_thrust,
                self.rocket.cg_from_gimbal,
            )
            aero_torque = np.zeros(3)
            grid_fins = 0.0
        elif self.phase == "phase2":
            pos_des = np.array([pos[0], pos[1], 0.0])
            vel_des = np.array([0.0, 0.0, vel[2]])
            
            # Pure ballistic fall - no lateral guidance
            throttle = 0.0
            delta_y, delta_z = 0.0, 0.0
            
            # Simple damping torque only
            aero_torque = -config.PHASE2_AERO_DAMP * omega
            grid_fins = 1.0
        elif self.phase == "phase3":
            pos_des = np.array([self.landing_target[0], self.landing_target[1], 0.0])
            vel_des = np.zeros(3)
            time_in_phase3 = t - self.phase3_start_time if self.phase3_start_time else 0.0
            throttle, delta_y, delta_z = phase3_controller(self.state, self.rocket, time_in_phase3, self.prev_gimbal)
            self.prev_gimbal = (delta_y, delta_z)
            aero_torque = np.zeros(3)
            grid_fins = 0.0
        else:  # landed
            pos_des = np.array([0.0, 0.0, 0.0])
            vel_des = np.zeros(3)
            throttle = 0.0
            delta_y, delta_z = 0.0, 0.0
            aero_torque = np.zeros(3)
            grid_fins = 0.0

        control = {
            "throttle": throttle,
            "gimbal_y": delta_y,
            "gimbal_z": delta_z,
            "aero_torque": aero_torque,
            "grid_fins": grid_fins,
        }
        return control, pos_des

    def step(self, t, dt):
        control, pos_des = self.compute_control(t)

        def ode(_t, y):
            return state_derivative(_t, y, control, self.rocket, phase=self.phase)

        sol = solve_ivp(ode, (t, t + dt), self.state, method="RK45", t_eval=[t + dt])
        self.state = sol.y[:, -1]

        self.history["t"].append(t + dt)
        self.history["state"].append(self.state.copy())
        self.history["control"].append(control)
        self.history["target"].append(pos_des)
        self.history["phase"].append(self.phase)

    def run(self, total_time, dt):
        frames = int(total_time / dt)
        for frame in range(frames):
            t = frame * dt
            self.step(t, dt)
            if t > 1.0 and self.state[2] <= 0.0:
                break
        return self.history
