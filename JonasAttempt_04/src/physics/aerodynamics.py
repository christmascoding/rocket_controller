"""
Aerodynamic force and moment computation.

All forces computed in **world frame**.
All moments computed in **body frame** (about CG).
"""
import numpy as np

from src.config import (S_REF, CL_ALPHA, CD_0, CD_ALPHA,
                         CP_OFFSET_ASCENT, CP_OFFSET_DESCENT)
from src.physics.environment import atmosphere_density
from src.math_utils import quat_to_dcm


def compute_aero(state: np.ndarray, phase: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (F_aero_world, M_aero_body).

    Parameters
    ----------
    state : 13-element state  [pos(3), vel(3), quat(4), omega(3)]
    phase : current mission phase string
    """
    pos  = state[0:3]
    vel  = state[3:6]
    quat = state[6:10]

    R = quat_to_dcm(quat)               # body → world
    altitude = max(pos[2], 0.0)
    rho = atmosphere_density(altitude)

    v_mag = np.linalg.norm(vel)
    if v_mag < 1.0:                      # negligible airspeed
        return np.zeros(3), np.zeros(3)

    v_hat = vel / v_mag                  # velocity unit vector (world)
    q_dyn = 0.5 * rho * v_mag**2        # dynamic pressure

    # --- velocity in body frame ---
    v_body = R.T @ vel
    v_lat  = np.array([v_body[0], v_body[1], 0.0])
    v_lat_mag = np.linalg.norm(v_lat)

    # angle of attack
    alpha = np.arctan2(v_lat_mag, abs(v_body[2]))

    # --- drag (always opposes velocity) ---
    Cd = CD_0 + CD_ALPHA * alpha**2
    F_drag = -q_dyn * S_REF * Cd * v_hat      # world frame

    # --- lift (perpendicular to v, toward body z) ---
    F_lift = np.zeros(3)
    if v_lat_mag > 0.01 and alpha > 1e-4:
        Cl = CL_ALPHA * alpha
        body_z_world = R @ np.array([0.0, 0.0, 1.0])
        # component of body-z perpendicular to velocity
        lift_dir = body_z_world - np.dot(body_z_world, v_hat) * v_hat
        ld_norm = np.linalg.norm(lift_dir)
        if ld_norm > 1e-8:
            lift_dir /= ld_norm
            F_lift = q_dyn * S_REF * Cl * lift_dir

    F_aero_world = F_drag + F_lift

    # --- moment about CG (body frame) ---
    cp_offset = _cp_offset_for_phase(phase)      # [0, 0, L_cp]
    F_aero_body = R.T @ F_aero_world
    M_aero_body = np.cross(cp_offset, F_aero_body)

    return F_aero_world, M_aero_body


def _cp_offset_for_phase(phase: str) -> np.ndarray:
    """CP position relative to CG in body frame."""
    if phase in ('2', '3', 'landed'):
        return np.array([0.0, 0.0, CP_OFFSET_DESCENT])   # stable (grid-fins)
    return np.array([0.0, 0.0, CP_OFFSET_ASCENT])         # slight instability
