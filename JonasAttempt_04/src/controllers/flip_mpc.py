"""
Model Predictive Controller for the **Phase 1c** boostback flip.

The manoeuvre is strongly nonlinear (180 ° rotation).  The MPC
optimises gimbal commands over a short horizon to steer the rocket
to retrograde orientation while respecting gimbal constraints and
rate limits.

Prediction model
----------------
Simplified rotational dynamics only (translation unaffected by the
~25 % throttle used during the flip).

State  :  q (4)  +  ω (3)  =  7-dim
Control:  [throttle (fixed), gimbal_y, gimbal_z]

Cost
----
J = w_angle · |angle_to_retrograde(q_N)|²
  + w_rate  · ‖ω_N‖²
  + w_ctrl  · Σ ‖u_k‖²
"""

import numpy as np
from scipy.optimize import minimize

from src.controllers.base import BaseController
from src.config import (
    MASS, F_MAX, GIMBAL_MAX, GIMBAL_TO_CG, AILERON_MAX_TORQUE,
    INERTIA, INERTIA_INV,
    FLIP_THROTTLE,
    MPC_HORIZON, MPC_DT, MPC_W_ANGLE, MPC_W_RATE, MPC_W_CONTROL,
    PID_KP_ATT, PID_KD_ATT,
)
from src.math_utils import (
    quat_normalize, quat_multiply, omega_to_quat_deriv,
    quat_to_dcm, safe_normalize, quat_error_vec, align_body_z_to,
)


class FlipMPC(BaseController): # controller class inherits from our base controller 
    """MPC-based 180° flip to retrograde orientation."""

    def __init__(self):
        self._prev_u = None                # warm-start

    def compute(self, t, state, info): # main function (gets called on every timestep)
        vel   = state[3:6] # takes 3,4,5 out of state array -> current velocities vx, vy, vz
        quat  = quat_normalize(state[6:10]) # gets current orientation (quations) from states 6,7,8,9
        omega = state[10:13] # gets current angular speed from 10,11,12 (wxwywz)

        debug = {} # for later

        # Target: body-z aligned with −v (retrograde)
        v_mag = np.linalg.norm(vel) # 3D pythagoras: betrag des geschwindigkeitsvektor berechnen (current speedbetrag in 3D)
        if v_mag > 5.0: #nur wenn wir schnell genug uns drehen berechnen wir hier den retrograde winkel
            retro_dir = -vel / v_mag
        else: # falls wir es aus gründen nicht tun sollten, setz retrograde stur nach oben damit wir nicht ins schleudern kommen -> sonst hüpft uns retrograde rum
            retro_dir = np.array([0.0, 0.0, 1.0])

        q_target = align_body_z_to(retro_dir) # align_body_z_to berechnet das ziel-quaternion aus unserer retrograde direction, also das reglerziel

        # ---------- run MPC optimisation -----------
        N = MPC_HORIZON # N = prädiktionshorizont (Schritte n)
        u0 = self._prev_u if self._prev_u is not None else np.zeros(2 * N) # wenn self._prev_u schon gegeben dann warmstart damit, ansonsten zeros (kaltstart)

        bounds = [(-GIMBAL_MAX, GIMBAL_MAX)] * (2 * N) # bounds für den gimbal dass der nicht zu stark eskalieren kann -> hilft bei dampening

        result = minimize( # DER MAGISCHE SCIPY SOLVER
            _mpc_cost, u0, # cost function + u0 warm startpoint falls er gegeben ist
            args=(quat, omega, q_target, N, MPC_DT), # werte die der solver für die cost funktion braucht: pos(quat, winkelgesch(omega), ziel(q_target))
            method='SLSQP', # Sequential Least Squares -> top für nichtlineare probleme, ist ein standardalgorithmus dafür
            bounds=bounds, # bounds halt
            options={'maxiter': 30, 'ftol': 1e-6}, # max iterations 30, wenn cost function 1e-(...) ist ist klein genug TODO TUNING!!!!!
        ) # 

        # als o
        u_opt = result.x # save result -> der plan diggi
        self._prev_u = np.concatenate([u_opt[2:], u_opt[-2:]])  # "löscht" das vergangene ergebnis für die nächst iteration, dann shiftet es alle ergebnisse nach links und zuletzt wird das letzte ergebniss einfach kopiert
        #=> wir kriegen perfekt den warm start für das nächste ergebnis

        # jetzt reduzieren wir den ganzen plan (alle steps in zukunft) auf die ersten zwei steps, cutten zur not nochmal gimbal werte raus
        gy = float(np.clip(u_opt[0], -GIMBAL_MAX, GIMBAL_MAX))
        gz = float(np.clip(u_opt[1], -GIMBAL_MAX, GIMBAL_MAX)) 

        throttle = FLIP_THROTTLE # constant throttle -> adjusting throttle with MPC dynamically is HARD :( -> wir lassen es

        # simpler P regler um rollen mit ailerons rauszukriegen falls wir roll kriegen (ist je nach mission immer anders)
        ail_z = float(np.clip(-500.0 * omega[2], -AILERON_MAX_TORQUE,
                              AILERON_MAX_TORQUE))
    

        control = np.array([throttle, gy, gz, 0.0, 0.0, ail_z]) # final output arrray with our valueeeeez

        debug['retro_dir']  = retro_dir
        debug['q_target']   = q_target
        debug['mpc_cost']   = result.fun
        return control, debug


# ---------- MPC internalzz ------------------------------------------------

def _predict_rotation(quat, omega, gy, gz, dt): # simuliert einen schritt in die zukunft
    """One Euler step of the simplified rotational dynamics."""
    T = FLIP_THROTTLE * F_MAX # tatsächliche Schubkraft (voll)
    sy, sz = np.sin(gy), np.sin(gz) # gimbal winkel gy und gz auf sy sz mit sin
    cyz = np.cos(gy) * np.cos(gz) # cyz ist cosinus multipliziert von gy gz
    F_body = T * np.array([sy, -sz, cyz]) # throttle force vector, split up in xyz components 
    r_engine = np.array([0.0, 0.0, -GIMBAL_TO_CG]) # vector from CG to gimbal
    M = np.cross(r_engine, F_body) # kreuzprodukt -> berechnung von momentauswirkung, basically 1:1 aus unserer physikenginge glaub

    Iomega = INERTIA @ omega # turning impulse matrix
    domega = INERTIA_INV @ (M - np.cross(omega, Iomega)) # angular acceleration with euler formula

    omega_new = omega + domega * dt # euler integration -> alterwert +(veränderung*zeit) = neuer wert aus VL
    dq = omega_to_quat_deriv(quat, omega) # veränderung quation magische berechnung
    quat_new = quat_normalize(quat + dq * dt) # neues quaternion nach zeitschritt, auf 1 betragn ormalsiiert

    return quat_new, omega_new


def _mpc_cost(u_flat, quat0, omega0, q_target, N, dt): # des koscht (wie gut oder schlecht ist unser berechneter plan)
    """Cost function evaluated by the optimiser."""
    q = quat0.copy() 
    w = omega0.copy()
    # copy copy copy baby

    #cost start at 0
    J_ctrl = 0.0
    for k in range(N): # for all steps to N
        gy = u_flat[2 * k]
        gz = u_flat[2 * k + 1]
        # get gimbal angles for this step
        J_ctrl += gy**2 + gz**2 # ^2 to punish the MPC from oversteering gimbals -> des muss viel koschta!
        q, w = _predict_rotation(q, w, gy, gz, dt) # step is simulated

    # terminal angle to target
    angle_err = _angle_to_target(q, q_target) # how far are we from q_target after all of this shit?

    J = (MPC_W_ANGLE * angle_err**2 # WOLLT IHR DIE TOTALE KOSTENFUNKTION?
         + MPC_W_RATE * np.dot(w, w)
         + MPC_W_CONTROL * J_ctrl)
    return J


def _angle_to_target(q, q_target): # quaternion angular hack, finds angle
    """Smallest rotation angle (rad) between two quaternions."""
    q_err = quat_multiply(
        np.array([q_target[0], -q_target[1], -q_target[2], -q_target[3]]), q)
    if q_err[0] < 0:
        q_err = -q_err
    return 2.0 * np.arccos(np.clip(q_err[0], -1.0, 1.0))
