import numpy as np
import cvxpy as cp
from dataclasses import dataclass


@dataclass
class MPCParams:
    horizon_steps: int
    dt: float
    weight_pos: float
    weight_vel: float
    weight_accel: float
    weight_jerk_thrust: float
    max_accel: float
    max_lateral_accel: float


class MPCController:
    def __init__(self, params: MPCParams):
        self.params = params
        self._build_problem()

    def _build_problem(self):
        N = self.params.horizon_steps
        dt = self.params.dt

        self.x0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.x_ref = cp.Parameter((3, N + 1))
        self.v_ref = cp.Parameter((3, N + 1))

        self.X = cp.Variable((3, N + 1))
        self.V = cp.Variable((3, N + 1))
        self.A = cp.Variable((3, N))

        constraints = [
            self.X[:, 0] == self.x0,
            self.V[:, 0] == self.v0,
        ]

        for k in range(N):
            constraints += [
                self.X[:, k + 1]
                == self.X[:, k] + self.V[:, k] * dt + 0.5 * self.A[:, k] * dt**2,
                self.V[:, k + 1] == self.V[:, k] + self.A[:, k] * dt,
            ]

            # Acceleration bounds
            constraints += [
                self.A[:, k] <= self.params.max_accel,
                self.A[:, k] >= -self.params.max_accel,
                self.A[0, k] <= self.params.max_lateral_accel,
                self.A[0, k] >= -self.params.max_lateral_accel,
                self.A[1, k] <= self.params.max_lateral_accel,
                self.A[1, k] >= -self.params.max_lateral_accel,
            ]

        cost = 0
        for k in range(N):
            pos_err = self.X[:, k] - self.x_ref[:, k]
            vel_err = self.V[:, k] - self.v_ref[:, k]
            cost += self.params.weight_pos * cp.sum_squares(pos_err)
            cost += self.params.weight_vel * cp.sum_squares(vel_err)
            cost += self.params.weight_accel * cp.sum_squares(self.A[:, k])
            if k > 0:
                # Jerk penalty only on thrust-axis acceleration (z)
                cost += self.params.weight_jerk_thrust * cp.sum_squares(
                    self.A[2, k] - self.A[2, k - 1]
                )

        # Terminal cost
        pos_err = self.X[:, N] - self.x_ref[:, N]
        vel_err = self.V[:, N] - self.v_ref[:, N]
        cost += self.params.weight_pos * cp.sum_squares(pos_err)
        cost += self.params.weight_vel * cp.sum_squares(vel_err)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def solve(self, position: np.ndarray, velocity: np.ndarray, x_ref: np.ndarray, v_ref: np.ndarray) -> np.ndarray:
        self.x0.value = position
        self.v0.value = velocity
        self.x_ref.value = x_ref
        self.v_ref.value = v_ref

        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            if self.A.value is None:
                return np.zeros(3)
            return np.array(self.A.value[:, 0]).astype(float)
        except Exception:
            return np.zeros(3)
