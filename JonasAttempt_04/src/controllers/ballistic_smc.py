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

    def compute(self, t, state, info): # gotta run each step
        pos   = state[0:3]
        vel   = state[3:6]
        quat  = quat_normalize(state[6:10])
        omega = state[10:13]
        # get the quark 3x pos, 3x vel, 4x quations and 3x angular speed

        debug = {}

        v_mag = np.linalg.norm(vel) # v betrag
        if v_mag > 5.0:
            retro_dir = -vel / v_mag # if we got some speed, point retrograde. if not, just point downwards
        else:
            retro_dir = np.array([0.0, 0.0, 1.0])

        q_target = align_body_z_to(retro_dir) # target angle quaternion 

        e_att = quat_error_vec(q_target, quat) # regelfehler-vektor -> bei kleinen vektoren annäherung benutzbar

        # sliding surface definition
        sigma = omega + SMC_LAMBDA * e_att # TODO magic number anpassen -> probier mal mit den unterschiedlichen surfaces rum

        # boundary layer with magic numbers, prevent strong bang bang responses
        def sat(s, phi=SMC_PHI):
            return np.clip(s / phi, -1.0, 1.0)

        # torque effectiveness scaling by dynamic pressure, lower atmosphere at pos[2] gives us more control
        q_dyn = dynamic_pressure(pos[2], v_mag) 
        q_ref = 10000.0              # reference q for full authority TODO magic number
        scale = np.clip(q_dyn / q_ref, 0.05, 1.0) # MAGIC NUMBER HARDCORE SH*T TODO

        torque = -SMC_K * sat(sigma) * AILERON_MAX_TORQUE * scale # regelgesetz (torque calc)

        torque = np.clip(torque, -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE) # aileron saturation, otherwice the SMC might go fkin insane

        control = np.array([0.0, 0.0, 0.0, torque[0], torque[1], torque[2]]) # mimomapping (thrust, gimbalxy, torquexyz)

        debug['retro_dir'] = retro_dir
        debug['att_err']   = e_att
        debug['sigma']     = sigma
        debug['q_dyn']     = q_dyn
        return control, debug
