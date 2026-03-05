"""
Quaternion and rotation utilities for 6-DOF simulation.

Convention
----------
Quaternion  q = [w, x, y, z]   (scalar-first, Hamilton).
q rotates vectors from **body frame → world frame**.
"""

import numpy as np

# ───────────────────────── quaternion algebra ─────────────────────────────

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product  q1 ⊗ q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate (= inverse for unit quaternions)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-14:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


# ──────────────────── conversions: quat ↔ DCM ↔ Euler ────────────────────

def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Unit quaternion → 3×3 Direction Cosine Matrix  (body → world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),        1 - 2*(x*x + y*y)],
    ])


def dcm_to_quat(R: np.ndarray) -> np.ndarray:
    """DCM → unit quaternion  (Shepperd's method)."""
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([w, x, y, z]))


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Quaternion → Euler angles  [roll φ, pitch θ, yaw ψ]  (ZYX convention).

    These are **display-only** — never fed back into the integrator.
    """
    w, x, y, z = q
    # roll  (around x)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr, cosr)
    # pitch (around y)
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # yaw   (around z)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny, cosy)
    return np.array([roll, pitch, yaw])


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler angles (ZYX)  →  quaternion."""
    cr, cp, cy = np.cos(roll / 2), np.cos(pitch / 2), np.cos(yaw / 2)
    sr, sp, sy = np.sin(roll / 2), np.sin(pitch / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


# ──────────────────────── vector rotation helpers ─────────────────────────

def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector *v* by quaternion *q*  (body → world)."""
    return quat_to_dcm(q) @ v


def quat_rotate_inv(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector *v* by the inverse of *q*  (world → body)."""
    return quat_to_dcm(q).T @ v


# ───────────────────── quaternion derivative / helpers ─────────────────────

def omega_to_quat_deriv(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """dq/dt = 0.5 · q ⊗ [0, ω_body]."""
    return 0.5 * quat_multiply(q, np.array([0.0, *omega_body]))


def axis_angle_to_quat(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-14:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    ha = angle / 2.0
    return np.array([np.cos(ha), *(axis * np.sin(ha))])


def quat_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Shortest rotation angle (rad) between two orientations."""
    q_err = quat_multiply(quat_conjugate(q1), q2)
    # Ensure positive scalar part for shortest path
    if q_err[0] < 0:
        q_err = -q_err
    return 2.0 * np.arccos(np.clip(q_err[0], -1.0, 1.0))


def quat_error_vec(q_desired: np.ndarray, q_actual: np.ndarray) -> np.ndarray:
    """Small-angle attitude error vector in **body frame**.

    For small errors:  error ≈ 2·[q_err.x, q_err.y, q_err.z]
    where q_err = q_desired* ⊗ q_actual.
    """
    q_err = quat_multiply(quat_conjugate(q_desired), q_actual)
    if q_err[0] < 0:
        q_err = -q_err          # shortest path
    return 2.0 * q_err[1:4]    # ≈ rotation-vector in body frame


def align_body_z_to(direction: np.ndarray) -> np.ndarray:
    """Quaternion that aligns body +z  with *direction* (world frame).

    Roll is implicitly zero (body-x stays as close to world-x as possible).
    """
    d = np.asarray(direction, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    d = d / n

    z0 = np.array([0.0, 0.0, 1.0])
    dot = np.clip(np.dot(z0, d), -1.0, 1.0)

    if dot > 0.9999:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -0.9999:
        # 180° rotation — pick an arbitrary perpendicular axis
        return axis_angle_to_quat(np.array([1.0, 0.0, 0.0]), np.pi)

    axis = np.cross(z0, d)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(dot)
    return axis_angle_to_quat(axis, angle)


# ──────────────────────── misc vector utilities ───────────────────────────

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def safe_normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)
