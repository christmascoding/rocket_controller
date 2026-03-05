import numpy as np

from src import config


def generate_parabolic_trajectory():
    t = np.arange(0.0, config.TRAJ_T_FINAL + config.SIM_DT, config.SIM_DT)
    gamma = np.linspace(0.0, 0.5 * np.pi, len(t))

    x_base = config.TRAJ_X_REF * (1.0 - np.cos(gamma))
    y_base = np.zeros_like(x_base)
    z = config.TRAJ_Z_REF * np.sin(gamma)

    # Keep the original half-circle in X-Z, add 45° direction in XY by setting y = x
    x = x_base
    y = x_base

    pos = np.stack([x, y, z], axis=1)

    vx = np.gradient(x, config.SIM_DT)
    vy = np.gradient(y, config.SIM_DT)
    vz = np.gradient(z, config.SIM_DT)
    vel = np.stack([vx, vy, vz], axis=1)

    return t, pos, vel


def sample_trajectory(t, traj_t, traj_pos, traj_vel):
    idx = np.searchsorted(traj_t, t, side="right") - 1
    idx = np.clip(idx, 0, len(traj_t) - 1)
    return traj_pos[idx], traj_vel[idx]
