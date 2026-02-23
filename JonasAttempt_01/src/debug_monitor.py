"""Comprehensive debug monitoring system for rocket controller"""
import numpy as np
from src.config import rad2deg


class DebugMonitor:
    """Monitors and logs all control and physics variables"""

    def __init__(self, every_n_steps: int = 5):
        self.every_n_steps = every_n_steps
        self.step_count = 0

    def log_step(self, step_info: dict):
        """Log a complete simulation step"""
        self.step_count += 1
        if self.step_count % self.every_n_steps != 0:
            return

        t = step_info["t"]
        
        # ===== STATE =====
        pos = step_info["position"]
        vel = step_info["velocity"]
        att = step_info["attitude"]  # [roll, pitch, yaw]
        ang_vel = step_info["angular_velocity"]
        
        # ===== TRAJECTORY REFERENCE =====
        ref_pos = step_info["ref_position"]
        ref_vel = step_info["ref_velocity"]
        ref_acc = step_info["ref_acceleration"]
        
        # ===== CONTROL COMMANDS =====
        desired_acc = step_info.get("desired_accel", np.array([0, 0, 0]))
        desired_pitch = step_info.get("desired_pitch", 0)
        desired_yaw = step_info.get("desired_yaw", 0)
        
        # ===== ACTUATOR STATES =====
        thrust_cmd = step_info.get("thrust_cmd", 0)
        thrust_actual = step_info.get("thrust_actual", 0)
        gimbal_cmd = step_info.get("gimbal_cmd", np.array([0, 0]))  # [pitch, yaw]
        gimbal_actual = step_info.get("gimbal_actual", np.array([0, 0]))
        
        # ===== MPC DEBUG =====
        mpc_enabled = step_info.get("mpc_enabled", False)
        mpc_cost = step_info.get("mpc_cost", None)
        
        # ===== ERRORS =====
        pos_error = np.linalg.norm(pos - ref_pos)
        vel_error = np.linalg.norm(vel - ref_vel)
        pitch_error = rad2deg(desired_pitch - att[1])
        yaw_error = rad2deg(desired_yaw - att[2])
        
        # ===== GIMBAL SATURATION =====
        gimbal_sat = np.abs(gimbal_actual)
        gimbal_saturated = np.any(gimbal_sat > 0.085)  # ~5 degrees
        
        # ===== ENERGY =====
        kinetic = 0.5 * step_info.get("mass", 50) * np.sum(vel**2)
        potential = step_info.get("mass", 50) * 9.81 * pos[2]
        
        # ===== BUILD DEBUG OUTPUT =====
        print(
            f"t={t:6.2f}s | "
            f"POS:[{pos[0]:7.1f},{pos[2]:7.1f}]m | "
            f"VEL:[{vel[0]:6.1f},{vel[2]:6.1f}]m/s | "
            f"ATT:[R{rad2deg(att[0]):6.1f}°,P{rad2deg(att[1]):6.1f}°,Y{rad2deg(att[2]):7.1f}°] | "
        )
        print(
            f"     "
            f"REF_POS:[{ref_pos[0]:7.1f},{ref_pos[2]:7.1f}]m | "
            f"ERR_POS:{pos_error:7.1f}m | "
            f"ERR_VEL:{vel_error:6.2f}m/s | "
        )
        print(
            f"     "
            f"THRUST: cmd={thrust_cmd:6.0f}N, actual={thrust_actual:6.0f}N | "
            f"GIMBAL: cmd=[{rad2deg(gimbal_cmd[0]):6.2f}°,{rad2deg(gimbal_cmd[1]):6.2f}°] | "
            f"actual=[{rad2deg(gimbal_actual[0]):6.2f}°,{rad2deg(gimbal_actual[1]):6.2f}°]"
        )
        if gimbal_saturated:
            print(f"     *** GIMBAL SATURATED ***")
        
        print(
            f"     "
            f"DES_ATT:[P{rad2deg(desired_pitch):6.1f}°,Y{rad2deg(desired_yaw):7.1f}°] | "
            f"ERR_ATT:[P{pitch_error:6.1f}°,Y{yaw_error:7.1f}°] | "
            f"ANGVEL:[R{rad2deg(ang_vel[0]):6.1f}°/s,P{rad2deg(ang_vel[1]):6.1f}°/s,Y{rad2deg(ang_vel[2]):6.1f}°/s]"
        )
        
        print(
            f"     "
            f"DESIRED_ACC:[{desired_acc[0]:6.2f},{desired_acc[2]:6.2f}]m/s² | "
            f"REF_ACC:[{ref_acc[0]:6.2f},{ref_acc[2]:6.2f}]m/s² | "
            f"ENERGY: KE={kinetic:8.0f}J, PE={potential:8.0f}J"
        )
        
        if mpc_enabled and mpc_cost is not None:
            print(f"     MPC COST: {mpc_cost:.2f}")
        
        print()


def attach_debug_info(simulator, step_dict):
    """Extract debug information from simulator state"""
    # Data from step is already complete
    debug_info = {
        "t": step_dict["time"],
        "position": step_dict["position"],
        "velocity": step_dict["velocity"],
        "attitude": step_dict["attitude"],
        "angular_velocity": step_dict["angular_velocity"],
        "ref_position": step_dict["pos_ref"],
        "ref_velocity": step_dict.get("ref_velocity", np.zeros(3)),
        "ref_acceleration": step_dict.get("ref_acceleration", np.zeros(3)),
        "mass": simulator.mass,
        "thrust_cmd": step_dict.get("thrust_cmd", 0),
        "thrust_actual": step_dict["thrust_n"],
        "gimbal_cmd": step_dict.get("gimbal_cmd", np.zeros(2)),
        "gimbal_actual": step_dict["gimbal_angles"],
        "desired_accel": step_dict.get("desired_accel", np.zeros(3)),
        "desired_pitch": step_dict.get("desired_pitch", 0),
        "desired_yaw": step_dict.get("desired_yaw", 0),
        "mpc_enabled": simulator.config.mpc.enable_mpc,
        "mpc_cost": step_dict.get("mpc_cost_total", None),
    }
    
    return debug_info
