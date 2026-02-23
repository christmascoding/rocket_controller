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
    weight_lateral_accel: float  # NEW: penalize lateral acceleration to reduce gimbal usage
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
            # Penalize lateral acceleration to reduce gimbal usage and roll risk
            cost += self.params.weight_lateral_accel * (self.A[0, k]**2 + self.A[1, k]**2)
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

    def solve(self, position: np.ndarray, velocity: np.ndarray, x_ref: np.ndarray, v_ref: np.ndarray) -> dict:
        self.x0.value = position
        self.v0.value = velocity
        self.x_ref.value = x_ref
        self.v_ref.value = v_ref

        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            
            if self.A.value is None or self.X.value is None or self.V.value is None:
                # Return safe defaults on failure
                N = self.params.horizon_steps
                return {
                    "accel_cmd": np.zeros(3),
                    "predicted_pos": np.zeros((3, N + 1)),
                    "predicted_vel": np.zeros((3, N + 1)),
                    "predicted_accel": np.zeros((3, N)),
                    "ref_pos": x_ref.copy(),
                    "ref_vel": v_ref.copy(),
                    "cost_total": 0.0,
                    "cost_pos": 0.0,
                    "cost_vel": 0.0,
                    "cost_accel": 0.0,
                    "cost_jerk": 0.0,
                    "solver_status": "failed",
                }
            
            # Extract solution
            accel_cmd = np.array(self.A.value[:, 0]).astype(float)
            predicted_pos = np.array(self.X.value).astype(float)
            predicted_vel = np.array(self.V.value).astype(float)
            predicted_accel = np.array(self.A.value).astype(float)
            
            # Compute cost components
            N = self.params.horizon_steps
            cost_pos = 0.0
            cost_vel = 0.0
            cost_accel = 0.0
            cost_jerk = 0.0
            
            for k in range(N):
                pos_err = predicted_pos[:, k] - x_ref[:, k]
                vel_err = predicted_vel[:, k] - v_ref[:, k]
                cost_pos += self.params.weight_pos * np.sum(pos_err**2)
                cost_vel += self.params.weight_vel * np.sum(vel_err**2)
                cost_accel += self.params.weight_accel * np.sum(predicted_accel[:, k]**2)
                if k > 0:
                    cost_jerk += self.params.weight_jerk_thrust * (predicted_accel[2, k] - predicted_accel[2, k - 1])**2
            
            # Terminal cost
            pos_err = predicted_pos[:, N] - x_ref[:, N]
            vel_err = predicted_vel[:, N] - v_ref[:, N]
            cost_pos += self.params.weight_pos * np.sum(pos_err**2)
            cost_vel += self.params.weight_vel * np.sum(vel_err**2)
            
            cost_total = cost_pos + cost_vel + cost_accel + cost_jerk
            
            return {
                "accel_cmd": accel_cmd,
                "predicted_pos": predicted_pos,
                "predicted_vel": predicted_vel,
                "predicted_accel": predicted_accel,
                "ref_pos": x_ref.copy(),
                "ref_vel": v_ref.copy(),
                "cost_total": float(cost_total),
                "cost_pos": float(cost_pos),
                "cost_vel": float(cost_vel),
                "cost_accel": float(cost_accel),
                "cost_jerk": float(cost_jerk),
                "solver_status": self.problem.status,
            }
        except Exception as e:
            N = self.params.horizon_steps
            return {
                "accel_cmd": np.zeros(3),
                "predicted_pos": np.zeros((3, N + 1)),
                "predicted_vel": np.zeros((3, N + 1)),
                "predicted_accel": np.zeros((3, N)),
                "ref_pos": x_ref.copy(),
                "ref_vel": v_ref.copy(),
                "cost_total": 0.0,
                "cost_pos": 0.0,
                "cost_vel": 0.0,
                "cost_accel": 0.0,
                "cost_jerk": 0.0,
                "solver_status": f"error: {str(e)}",
            }
