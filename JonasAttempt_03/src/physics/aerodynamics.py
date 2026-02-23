import numpy as np

from src import config


def compute_aero_forces_moments(v_world, dcm_body_to_world, rocket, phase, control):
    """
    Returns aerodynamic force (world) and moment (body).
    Body Z is rocket nose axis.
    """
    v_mag = np.linalg.norm(v_world)
    if v_mag < 1e-3:
        return np.zeros(3), np.zeros(3)

    v_body = dcm_body_to_world.T @ v_world
    v_hat_body = v_body / (np.linalg.norm(v_body) + 1e-9)

    body_z = np.array([0.0, 0.0, 1.0])
    cos_alpha = np.clip(np.dot(v_hat_body, body_z), -1.0, 1.0)
    alpha = np.arccos(cos_alpha)

    q = 0.5 * config.RHO * v_mag ** 2

    # Phase 2: drag only, no lift
    if phase == "phase2":
        cd = config.CD0 + config.CD_ALPHA * alpha ** 2
        drag_mag = q * config.S_REF * cd
        drag_dir_body = -v_hat_body
        f_aero_body = drag_mag * drag_dir_body
    else:
        cl = config.CL_ALPHA * alpha
        cd = config.CD0 + config.CD_ALPHA * alpha ** 2

        lift_mag = q * config.S_REF * cl
        drag_mag = q * config.S_REF * cd

        drag_dir_body = -v_hat_body
        lift_dir_body = np.cross(np.cross(v_hat_body, body_z), v_hat_body)
        lift_norm = np.linalg.norm(lift_dir_body)
        if lift_norm > 1e-6:
            lift_dir_body /= lift_norm
        else:
            lift_dir_body = np.zeros(3)

        f_aero_body = drag_mag * drag_dir_body + lift_mag * lift_dir_body

    # Moment from CP offset (phase-dependent)
    grid_fins = float(control.get("grid_fins", 0.0))
    r_cp = rocket.cp_offset_for_phase(phase, grid_fins)
    m_aero_body = np.cross(r_cp, f_aero_body)

    f_aero_world = dcm_body_to_world @ f_aero_body
    return f_aero_world, m_aero_body
