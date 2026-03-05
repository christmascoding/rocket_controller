"""
6-DOF rigid-body dynamics  (the Plant / Anlagenmodell).

State vector (13 elements)
--------------------------
[x, y, z, vx, vy, vz, qw, qx, qy, qz, ωx, ωy, ωz]

Control vector (6 elements)
---------------------------
[throttle, gimbal_y, gimbal_z, aileron_τx, aileron_τy, aileron_τz]

Integration is done externally (RK4) — this module only
computes the state derivative  ẋ = f(x, u).
"""

import numpy as np

from src.config import MASS, INERTIA, INERTIA_INV, AILERON_MAX_TORQUE
from src.math_utils import quat_to_dcm, omega_to_quat_deriv, quat_normalize
from src.physics.environment import gravity_force
from src.physics.propulsion import compute_thrust
from src.physics.aerodynamics import compute_aero


def state_derivative(state: np.ndarray,
                     control: np.ndarray,
                     phase: str) -> np.ndarray:
    """Compute ẋ for the 13-element state.

    Returns
    -------
    dstate : (13,) array — time-derivative of the state.
    Also see `compute_forces_moments` for the force / moment breakdown
    (useful for logging).
    """
    _, dstate = _dynamics_core(state, control, phase)
    return dstate


def compute_forces_moments(state: np.ndarray,
                           control: np.ndarray,
                           phase: str) -> dict:
    """Return a dict with the full force / moment breakdown (for logging)."""
    info, _ = _dynamics_core(state, control, phase, full_info=True)
    return info


# ──────────────────────────── internal core ───────────────────────────────

def _dynamics_core(state, control, phase, full_info=False):
    pos   = state[0:3]
    vel   = state[3:6]
    quat  = quat_normalize(state[6:10])
    omega = state[10:13]

    R = quat_to_dcm(quat)  # body → world

    # ---- forces ----
    F_grav = gravity_force(MASS)

    throttle, gy, gz = control[0], control[1], control[2]
    aileron = control[3:6]

    F_thrust_body, M_thrust_body = compute_thrust(throttle, gy, gz)
    F_thrust_world = R @ F_thrust_body

    F_aero_world, M_aero_body = compute_aero(state, phase)

    F_total_world = F_grav + F_thrust_world + F_aero_world

    # ---- moments (body frame) ----
    # Clamp aileron torques
    ail = np.clip(aileron, -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE)
    M_aileron = np.array([ail[0], ail[1], ail[2]])

    M_total_body = M_thrust_body + M_aero_body + M_aileron

    # ---- derivatives ----
    d_pos  = vel
    d_vel  = F_total_world / MASS
    d_quat = omega_to_quat_deriv(quat, omega)
    #  Euler equation:  I ω̇ = M − ω × (Iω)
    Iomega = INERTIA @ omega
    d_omega = INERTIA_INV @ (M_total_body - np.cross(omega, Iomega))

    dstate = np.concatenate([d_pos, d_vel, d_quat, d_omega])

    if full_info:
        info = dict(
            F_gravity=F_grav,
            F_thrust_body=F_thrust_body,
            F_thrust_world=F_thrust_world,
            F_aero_world=F_aero_world,
            M_thrust_body=M_thrust_body,
            M_aero_body=M_aero_body,
            M_aileron=M_aileron,
            M_total_body=M_total_body,
        )
        return info, dstate
    return None, dstate
