import numpy as np


def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi] range."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def euler_to_dcm(phi, theta, psi):
    """
    Direction cosine matrix from body to world using ZYX convention.
    Body Z is rocket nose axis.
    """
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    # ZYX rotation
    return np.array(
        [
            [cpsi * cth, cpsi * sth * sphi - spsi * cphi, cpsi * sth * cphi + spsi * sphi],
            [spsi * cth, spsi * sth * sphi + cpsi * cphi, spsi * sth * cphi - cpsi * sphi],
            [-sth, cth * sphi, cth * cphi],
        ]
    )


def body_z_axis_in_world(phi, theta, psi):
    dcm = euler_to_dcm(phi, theta, psi)
    return dcm[:, 2]


def omega_to_euler_rates(phi, theta, omega_body):
    p, q, r = omega_body
    cth = np.cos(theta)
    sth = np.sin(theta)
    tth = np.tan(theta)
    cphi = np.cos(phi)
    sphi = np.sin(phi)

    # Avoid singularities by bounding cos(theta)
    cth = np.clip(cth, 1e-4, None)

    return np.array(
        [
            p + q * sphi * tth + r * cphi * tth,
            q * cphi - r * sphi,
            q * sphi / cth + r * cphi / cth,
        ]
    )
