from dataclasses import dataclass, field
import numpy as np


@dataclass
class ThrustActuator:
    time_constant_s: float
    rate_limit_n_per_s: float
    thrust_n: float = 0.0

    def update(self, thrust_cmd_n: float, dt: float) -> float:
        # First-order lag
        target = thrust_cmd_n
        alpha = dt / max(self.time_constant_s, 1e-6)
        desired = self.thrust_n + alpha * (target - self.thrust_n)

        # Rate limit
        max_delta = self.rate_limit_n_per_s * dt
        delta = np.clip(desired - self.thrust_n, -max_delta, max_delta)
        self.thrust_n += delta
        return self.thrust_n


@dataclass
class GimbalActuator:
    max_angle_rad: float
    rate_limit_rad_per_s: float
    accel_limit_rad_per_s2: float
    angle_rad: np.ndarray = field(default_factory=lambda: np.zeros(2))  # [pitch, yaw]
    rate_rad_per_s: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def update(self, cmd_rad: np.ndarray, dt: float) -> np.ndarray:
        # Clamp command to max angle
        cmd = np.clip(cmd_rad, -self.max_angle_rad, self.max_angle_rad)

        # Desired rate to reach command
        rate_cmd = (cmd - self.angle_rad) / max(dt, 1e-6)
        rate_cmd = np.clip(rate_cmd, -self.rate_limit_rad_per_s, self.rate_limit_rad_per_s)

        # Apply accel limits to rate
        max_rate_delta = self.accel_limit_rad_per_s2 * dt
        rate_delta = np.clip(rate_cmd - self.rate_rad_per_s, -max_rate_delta, max_rate_delta)
        self.rate_rad_per_s += rate_delta

        # Integrate angle with rate limits
        self.rate_rad_per_s = np.clip(self.rate_rad_per_s, -self.rate_limit_rad_per_s, self.rate_limit_rad_per_s)
        self.angle_rad += self.rate_rad_per_s * dt
        self.angle_rad = np.clip(self.angle_rad, -self.max_angle_rad, self.max_angle_rad)
        return self.angle_rad.copy()
