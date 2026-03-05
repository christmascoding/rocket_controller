"""
Landing controller for **Phase 3** — powered hoverslam / suicide burn.

Architecture
------------
1. **Vertical channel** : Suicide-burn throttle schedule computed from
   the analytic constant-deceleration profile.
2. **Lateral channel**  : PD on (x, y) position toward the landing target,
   producing desired lateral acceleration → desired tilt angles.
3. **Attitude tracking**: Linearised LQR (re-linearised each step around
   the current thrust) maps small-angle attitude + rate errors to gimbal.
   Falls back to a PD inner loop when scipy is unavailable or when the
   linearisation is ill-conditioned.

The 10-state reduced model for linearised LQR:
    δx = [Δx, Δy, Δz, Δvx, Δvy, Δvz, Δφ, Δθ, Δp, Δq]
    δu = [Δthrottle, Δgimbal_y, Δgimbal_z]
"""

import numpy as np

from src.controllers.base import BaseController
from src.config import (
    MASS, G, F_MAX, GIMBAL_MAX, GIMBAL_TO_CG,
    AILERON_MAX_TORQUE, LANDING_TARGET,
    I_XX, I_YY,
    LQR_Q_DIAG, LQR_R_DIAG,
)
from src.math_utils import (
    quat_normalize, quat_error_vec, align_body_z_to,
    quat_to_dcm, safe_normalize, clamp,
)

try:
    from scipy.linalg import solve_continuous_are
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


class LandingLQR(BaseController):
    """Suicide-burn + linearised LQR tracking."""

    def __init__(self):
        self._burn_started = False
        self._K = None                    # cached LQR gain

    def compute(self, t, state, info):
        pos   = state[0:3]
        vel   = state[3:6]
        quat  = quat_normalize(state[6:10])
        omega = state[10:13]

        debug = {}

        # ---- 1. vertical throttle (suicide-burn profile) ------------------
        alt = pos[2]
        vz  = vel[2]                       # negative when falling
        a_max = F_MAX / MASS - G           # max available deceleration
        speed_down = max(-vz, 0.0)         # positive speed toward ground
        d_stop = speed_down**2 / (2.0 * a_max) if a_max > 0.1 else 1e9

        # desired vertical accel: constant deceleration to zero
        if alt < 0.5:
            # very close to ground — gentle thrust
            accel_z_des = -vz * 3.0 + G
        elif speed_down > 1.0:
            # proportional-ish to velocity
            time_to_ground = alt / speed_down if speed_down > 0.5 else 1e6
            accel_z_des = speed_down / max(time_to_ground, 0.5) + G
            # cap at physical max
            accel_z_des = min(accel_z_des, F_MAX / MASS)
        else:
            # hovering / slow — maintain altitude
            accel_z_des = G + 2.0 * (0.0 - vz) + 0.5 * (5.0 - alt)

        throttle_nom = np.clip(accel_z_des * MASS / F_MAX, 0.0, 1.0)

        # ---- 2. lateral PD toward landing target --------------------------
        target = LANDING_TARGET
        pos_err_xy = target[:2] - pos[:2]
        vel_xy     = vel[:2]

        Kp_lat = 0.15
        Kd_lat = 0.6
        accel_xy_des = Kp_lat * pos_err_xy - Kd_lat * vel_xy

        # Desired tilt angles from lateral accel
        # For small angles:  a_x ≈ g·θ,  a_y ≈ −g·φ
        T_current = max(throttle_nom * F_MAX, MASS * G * 0.3)
        theta_des = np.clip(MASS * accel_xy_des[0] / T_current,
                            -0.3, 0.3)
        phi_des   = np.clip(-MASS * accel_xy_des[1] / T_current,
                            -0.3, 0.3)

        # Desired direction for body z
        desired_dir = np.array([np.sin(theta_des),
                                -np.sin(phi_des),
                                np.cos(theta_des) * np.cos(phi_des)])
        n = np.linalg.norm(desired_dir)
        if n > 1e-6:
            desired_dir /= n
        else:
            desired_dir = np.array([0.0, 0.0, 1.0])

        q_des = align_body_z_to(desired_dir)

        # ---- 3. attitude → gimbal  (LQR or PD fallback) ------------------
        att_err = quat_error_vec(q_des, quat)   # body frame

        # Recompute LQR gain periodically
        K = self._compute_lqr_gain(throttle_nom)

        if K is not None:
            # LQR: u = −K x  produces corrective gimbal commands
            # (sign absorbed in B matrix of the linearisation)
            x_lqr = np.array([att_err[0], att_err[1], omega[0], omega[1]])
            u_lqr = -K @ x_lqr
            gimbal_y = float(np.clip(u_lqr[0], -GIMBAL_MAX, GIMBAL_MAX))
            gimbal_z = float(np.clip(u_lqr[1], -GIMBAL_MAX, GIMBAL_MAX))
        else:
            # PD fallback (same sign convention as ascent PID:
            # positive gimbal → negative moment → corrective)
            Kp = 0.5
            Kd = 0.25
            gimbal_y = float(np.clip(Kp * att_err[1] + Kd * omega[1],
                                     -GIMBAL_MAX, GIMBAL_MAX))
            gimbal_z = float(np.clip(Kp * att_err[0] + Kd * omega[0],
                                     -GIMBAL_MAX, GIMBAL_MAX))

        # Roll damping
        ail_z = float(np.clip(-800.0 * att_err[2] - 400.0 * omega[2],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))

        control = np.array([throttle_nom, gimbal_y, gimbal_z,
                            0.0, 0.0, ail_z])

        debug['desired_dir'] = desired_dir
        debug['att_err']     = att_err
        debug['throttle']    = throttle_nom
        debug['d_stop']      = d_stop
        debug['alt']         = alt
        return control, debug

    # ------------------------------------------------------------------
    def _compute_lqr_gain(self, throttle_nom):
        """4-state LQR: [att_err_x, att_err_y, omega_x, omega_y]
        Controls: [gimbal_y, gimbal_z].
        """
        if not _HAS_SCIPY:
            return None
        try:
            T = max(throttle_nom * F_MAX, MASS * G * 0.3)
            L = GIMBAL_TO_CG
            # Linearised plant around vertical orientation:
            #   Gimbal creates moment: M_y = −L·T·gy,  M_x = −L·T·(−gz)
            #   So: dω_x/dt = L·T·(−gz) / I_xx  →  B maps u=[gy,gz]
            #       dω_y/dt = −L·T·gy / I_yy
            #   But our error convention (quat_error_vec) means the
            #   controller output u = −Kx should produce corrective
            #   gimbal commands WITH the geometric sign inversion.
            #   We absorb the sign into B so that  u = −Kx  "just works".
            b_x = L * T / I_XX         # maps gz → α_x  (with sign absorbed)
            b_y = L * T / I_YY         # maps gy → α_y

            A = np.array([
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ], dtype=float)

            # B maps [gimbal_y, gimbal_z] → [dω_x/dt, dω_y/dt]
            # Consistent with u = +K @ error  (corrective)
            B = np.array([
                [0,     0],
                [0,     0],
                [0,   b_x],
                [b_y,   0],
            ], dtype=float)

            Q = np.diag([40.0, 40.0, 4.0, 4.0])
            R = np.diag([1.0, 1.0])

            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ P
            self._K = K
            return K
        except np.linalg.LinAlgError:
            return self._K
        except ValueError:
            return self._K
        except Exception:
            return self._K        # return cached or None
