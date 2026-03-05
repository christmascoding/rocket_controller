"""
Sliding-Mode Controller for **Phase 2** — ballistic descent.

No engine — only aerodynamic control surfaces (ailerons / grid-fins)
maintain the retrograde attitude as the dynamic pressure changes
massively during the fall.

Sliding surface
---------------
σ = ω_err + λ · e_att

where  e_att = attitude error toward retrograde,  ω_err = ω (want ω → 0).

Control law (with boundary layer for chatter suppression):
    τ = −K · sat(σ / Φ)  · q_dyn_scale

The available torque is scaled by dynamic pressure because the
grid-fin effectiveness is proportional to q = ½ρv².
"""

import numpy as np

from src.controllers.base import BaseController
from src.config import (
    AILERON_MAX_TORQUE, SMC_LAMBDA, SMC_K, SMC_PHI,
)
from src.math_utils import (
    quat_normalize, quat_error_vec, align_body_z_to, safe_normalize,
)
from src.physics.environment import dynamic_pressure


class BallisticSMC(BaseController):
    """Sliding-mode attitude controller — ailerons only."""

    def compute(self, t, state, info):
        pos   = state[0:3]
        vel   = state[3:6]
        quat  = quat_normalize(state[6:10])
        omega = state[10:13]

        debug = {}

        # Target: retrograde (body-z → −v)
        v_mag = np.linalg.norm(vel)
        if v_mag > 5.0:
            retro_dir = -vel / v_mag
        else:
            retro_dir = np.array([0.0, 0.0, 1.0])

        q_target = align_body_z_to(retro_dir)

        # attitude error (body frame, small-angle approx)
        e_att = quat_error_vec(q_target, quat)

        # sliding surface
        sigma = omega + SMC_LAMBDA * e_att

        # saturation function (boundary layer)
        def sat(s, phi=SMC_PHI):
            return np.clip(s / phi, -1.0, 1.0)

        # torque effectiveness scaling by dynamic pressure
        q_dyn = dynamic_pressure(pos[2], v_mag)
        q_ref = 10000.0              # reference q for full authority
        scale = np.clip(q_dyn / q_ref, 0.05, 1.0)

        torque = -SMC_K * sat(sigma) * AILERON_MAX_TORQUE * scale

        torque = np.clip(torque, -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE)

        control = np.array([0.0, 0.0, 0.0, torque[0], torque[1], torque[2]])

        debug['retro_dir'] = retro_dir
        debug['att_err']   = e_att
        debug['sigma']     = sigma
        debug['q_dyn']     = q_dyn
        return control, debug
