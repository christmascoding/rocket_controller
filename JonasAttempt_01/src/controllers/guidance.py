import numpy as np
from dataclasses import dataclass


@dataclass
class GuidanceController:
    kp: float
    kd: float
    max_desired_accel: float

    def compute_desired_accel(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        pos_ref: np.ndarray,
        vel_ref: np.ndarray,
        acc_ref: np.ndarray,
    ) -> np.ndarray:
        pos_error = pos_ref - pos
        vel_error = vel_ref - vel
        desired = acc_ref + self.kp * pos_error + self.kd * vel_error

        mag = np.linalg.norm(desired)
        if mag > self.max_desired_accel:
            desired = desired * (self.max_desired_accel / max(mag, 1e-9))
        return desired
