from dataclasses import dataclass
import numpy as np


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integrator: np.ndarray
    integrator_limit: float = 1e6

    def reset(self):
        self.integrator[:] = 0.0

    def update(self, error: np.ndarray, error_rate: np.ndarray, dt: float) -> np.ndarray:
        self.integrator += error * dt
        norm = np.linalg.norm(self.integrator)
        if norm > self.integrator_limit:
            self.integrator *= self.integrator_limit / max(norm, 1e-9)
        return self.kp * error + self.ki * self.integrator + self.kd * error_rate
