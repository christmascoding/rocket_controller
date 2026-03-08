"""
Cascaded PD controller for **Phase 1a** (powered ascent) and
**Phase 1b** (coast).

Outer  loop :  position error  → desired velocity  → desired accel / force
Middle loop :  desired force   → desired orientation + throttle
Inner  loop :  attitude error  → gimbal + aileron commands
"""

import numpy as np

from src.controllers.base import BaseController
from src.config import (
    MASS, G, F_MAX, GIMBAL_MAX, AILERON_MAX_TORQUE,
    PID_KP_POS, PID_KD_VEL, PID_KP_ATT, PID_KD_ATT,
    ROLL_KP, ROLL_KD,
)
from src.math_utils import (
    quat_normalize, quat_error_vec, align_body_z_to,
    quat_to_dcm, safe_normalize,
)

# 3d pid mit quaterions und dem regelfehler für ascent
# TODO der hat abweichungen vom kurs
class AscentPID(BaseController):
    """
    Phase 1a : full thrust, cascade PD tracks reference trajectory.
    Phase 1b : throttle = 0, only ailerons maintain prograde orientation.
    """

    def compute(self, t, state, info):
        phase = info['phase']
        pos   = state[0:3]
        vel   = state[3:6]
        quat  = quat_normalize(state[6:10])
        omega = state[10:13]

        debug = {}

        if phase == '1a':
            return self._powered_ascent(t, pos, vel, quat, omega, info, debug)
        else:  # '1b'
            return self._coast(t, pos, vel, quat, omega, info, debug)

    # phase 1a

    def _powered_ascent(self, t, pos, vel, quat, omega, info, debug):
        ref_pos = info.get('ref_pos', pos)
        ref_vel = info.get('ref_vel', vel)

        # outer loop: position -> desired force
        pos_err  = ref_pos - pos
        vel_des  = ref_vel + PID_KP_POS * pos_err
        vel_err  = vel_des - vel 
        accel_des = PID_KD_VEL * vel_err # where should force point?
        # gravity compensation
        force_des = MASS * (accel_des + np.array([0.0, 0.0, G]))  # thats my G

        # middle loop: force -> attitude + throttle
        force_mag = np.linalg.norm(force_des)
        throttle  = np.clip(force_mag / F_MAX, 0.05, 1.0)

        if force_mag > 1.0:
            desired_dir = force_des / force_mag
        else:
            desired_dir = np.array([0.0, 0.0, 1.0])

        desired_quat = align_body_z_to(desired_dir)

        # inner loop: attitude -> gimbal

        # Sign convention: gimbal -> moment has a geometric sign inversion
        #   M_y = −L·T·sin(gy),  M_x = +L·T·(−sin(gz))
        # so POSITIVE gimbal_y creates NEGATIVE M_y.  To correct a positive attitude error (actual ahead of desired), we need negative alpha, sooo positive gimbal ->  + KP·e  (not −KP·e!) 

        att_err = quat_error_vec(desired_quat, quat)   # body frame
        gimbal_y = float(np.clip(PID_KP_ATT * att_err[1] + PID_KD_ATT * omega[1],
                                 -GIMBAL_MAX, GIMBAL_MAX))
        gimbal_z = float(np.clip(PID_KP_ATT * att_err[0] + PID_KD_ATT * omega[0],
                                 -GIMBAL_MAX, GIMBAL_MAX))
        # roll damping via ailerons (direct torque — no geometric inversion,
        # standard negative-feedback sign −KP·e is correct here, but not for gimbal
        ail_z = float(np.clip(-ROLL_KP * att_err[2] - ROLL_KD * omega[2],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))

        control = np.array([throttle, gimbal_y, gimbal_z, 0.0, 0.0, ail_z])

        debug['desired_dir']  = desired_dir
        debug['pos_err']      = pos_err
        debug['att_err']      = att_err
        debug['force_des']    = force_des
        return control, debug

    # phase 1b

    def _coast(self, t, pos, vel, quat, omega, info, debug):
        # maintain prograde orientation (align body-z with velocity)
        v_mag = np.linalg.norm(vel)
        if v_mag > 5.0:
            desired_dir = vel / v_mag
        else:
            desired_dir = np.array([0.0, 0.0, 1.0])

        desired_quat = align_body_z_to(desired_dir)
        att_err = quat_error_vec(desired_quat, quat)

        # ailerons only
        # re-use roll axis gains lol TODO HACKY
        ail_x = float(np.clip(-ROLL_KP * att_err[0] - ROLL_KD * omega[0],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))
        ail_y = float(np.clip(-ROLL_KP * att_err[1] - ROLL_KD * omega[1],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))
        ail_z = float(np.clip(-ROLL_KP * att_err[2] - ROLL_KD * omega[2],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))

        control = np.array([0.0, 0.0, 0.0, ail_x, ail_y, ail_z])

        debug['desired_dir'] = desired_dir
        debug['att_err']     = att_err
        return control, debug
