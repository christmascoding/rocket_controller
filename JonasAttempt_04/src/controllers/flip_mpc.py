"""
Model Predictive Controller for the **Phase 1c** boostback flip.

The manoeuvre is strongly nonlinear (180 ° rotation).  The MPC
optimises gimbal commands over a short horizon to steer the rocket
to retrograde orientation while respecting gimbal constraints and
rate limits.

Prediction model
----------------
Simplified rotational dynamics only (translation unaffected by the
~25 % throttle used during the flip).

State  :  q (4)  +  ω (3)  =  7-dim
Control:  [throttle (fixed), gimbal_y, gimbal_z]

Cost
----
J = w_angle · |angle_to_retrograde(q_N)|²
  + w_rate  · ‖ω_N‖²
  + w_ctrl  · Σ ‖u_k‖²
"""

import numpy as np
from scipy.optimize import minimize

from src.controllers.base import BaseController
from src.config import (
    MASS, F_MAX, GIMBAL_MAX, GIMBAL_TO_CG, AILERON_MAX_TORQUE,
    INERTIA, INERTIA_INV,
    FLIP_THROTTLE,
    MPC_HORIZON, MPC_DT, MPC_W_ANGLE, MPC_W_RATE, MPC_W_CONTROL,
    PID_KP_ATT, PID_KD_ATT,
)
from src.math_utils import (
    quat_normalize, quat_multiply, omega_to_quat_deriv,
    quat_to_dcm, safe_normalize, quat_error_vec, align_body_z_to,
)


class FlipMPC(BaseController):
    """MPC-based 180° flip to retrograde orientation."""

    def __init__(self):
        self._prev_u = None                # warm-start

    def compute(self, t, state, info):
        vel   = state[3:6]
        quat  = quat_normalize(state[6:10])
        omega = state[10:13]

        debug = {}

        # Target: body-z aligned with −v (retrograde)
        v_mag = np.linalg.norm(vel)
        if v_mag > 5.0:
            retro_dir = -vel / v_mag
        else:
            retro_dir = np.array([0.0, 0.0, 1.0])

        q_target = align_body_z_to(retro_dir)

        # ---------- run MPC optimisation -----------
        N = MPC_HORIZON
        u0 = self._prev_u if self._prev_u is not None else np.zeros(2 * N)

        bounds = [(-GIMBAL_MAX, GIMBAL_MAX)] * (2 * N)

        result = minimize(
            _mpc_cost, u0,
            args=(quat, omega, q_target, N, MPC_DT),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 30, 'ftol': 1e-6},
        )

        u_opt = result.x
        self._prev_u = np.concatenate([u_opt[2:], u_opt[-2:]])  # shift

        gy = float(np.clip(u_opt[0], -GIMBAL_MAX, GIMBAL_MAX))
        gz = float(np.clip(u_opt[1], -GIMBAL_MAX, GIMBAL_MAX))

        throttle = FLIP_THROTTLE

        # Roll damping via ailerons
        ail_z = float(np.clip(-500.0 * omega[2], -AILERON_MAX_TORQUE,
                              AILERON_MAX_TORQUE))

        control = np.array([throttle, gy, gz, 0.0, 0.0, ail_z])

        debug['retro_dir']  = retro_dir
        debug['q_target']   = q_target
        debug['mpc_cost']   = result.fun
        return control, debug


# ──────────────────── MPC internals ───────────────────────────────────────

def _predict_rotation(quat, omega, gy, gz, dt):
    """One Euler step of the simplified rotational dynamics."""
    T = FLIP_THROTTLE * F_MAX
    sy, sz = np.sin(gy), np.sin(gz)
    cyz = np.cos(gy) * np.cos(gz)
    F_body = T * np.array([sy, -sz, cyz])
    r_engine = np.array([0.0, 0.0, -GIMBAL_TO_CG])
    M = np.cross(r_engine, F_body)

    Iomega = INERTIA @ omega
    domega = INERTIA_INV @ (M - np.cross(omega, Iomega))

    omega_new = omega + domega * dt
    dq = omega_to_quat_deriv(quat, omega)
    quat_new = quat_normalize(quat + dq * dt)

    return quat_new, omega_new


def _mpc_cost(u_flat, quat0, omega0, q_target, N, dt):
    """Cost function evaluated by the optimiser."""
    q = quat0.copy()
    w = omega0.copy()

    J_ctrl = 0.0
    for k in range(N):
        gy = u_flat[2 * k]
        gz = u_flat[2 * k + 1]
        J_ctrl += gy**2 + gz**2
        q, w = _predict_rotation(q, w, gy, gz, dt)

    # terminal angle to target
    angle_err = _angle_to_target(q, q_target)

    J = (MPC_W_ANGLE * angle_err**2
         + MPC_W_RATE * np.dot(w, w)
         + MPC_W_CONTROL * J_ctrl)
    return J


def _angle_to_target(q, q_target):
    """Smallest rotation angle (rad) between two quaternions."""
    q_err = quat_multiply(
        np.array([q_target[0], -q_target[1], -q_target[2], -q_target[3]]), q)
    if q_err[0] < 0:
        q_err = -q_err
    return 2.0 * np.arccos(np.clip(q_err[0], -1.0, 1.0))
