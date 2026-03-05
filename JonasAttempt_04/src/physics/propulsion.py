"""
Propulsion — thrust vector and gimbal mechanics.

Returns force in **body frame** and moment about CG in **body frame**.
"""
import numpy as np

from src.config import F_MAX, GIMBAL_MAX, GIMBAL_TO_CG


def compute_thrust(throttle: float,
                   gimbal_y: float,
                   gimbal_z: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute thrust force & moment in **body frame**.

    Parameters
    ----------
    throttle  : 0 … 1
    gimbal_y  : nozzle deflection about body-y  (tilts thrust in x-z plane)
    gimbal_z  : nozzle deflection about body-x  (tilts thrust in y-z plane)

    Returns
    -------
    F_body : (3,) thrust force in body frame
    M_body : (3,) moment about CG in body frame
    """
    throttle = float(np.clip(throttle, 0.0, 1.0))
    gimbal_y = float(np.clip(gimbal_y, -GIMBAL_MAX, GIMBAL_MAX))
    gimbal_z = float(np.clip(gimbal_z, -GIMBAL_MAX, GIMBAL_MAX))

    T = throttle * F_MAX

    # Thrust direction in body frame
    #   Nominal: +z_B (toward nose).
    #   gimbal_y rotates about y → component in x-z plane
    #   gimbal_z rotates about x → component in y-z plane
    sy, sz = np.sin(gimbal_y), np.sin(gimbal_z)
    cyz = np.cos(gimbal_y) * np.cos(gimbal_z)
    F_body = T * np.array([sy, -sz, cyz])

    # Engine is at −z_B from CG
    r_engine = np.array([0.0, 0.0, -GIMBAL_TO_CG])
    M_body = np.cross(r_engine, F_body)

    return F_body, M_body
