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
    _HAS_SCIPY = False # redundant shit, can remove this


class LandingLQR(BaseController):
    """Suicide-burn + linearised LQR tracking.""" # suicide burn is a bad word! -> ich weiß aber hoverslam klingt nicht so cool

    def __init__(self):
        self._burn_started = False # burn baby burn
        self._K = None # cached LQR gain for later

    def compute(self, t, state, info): # controller loopity loop
        pos   = state[0:3] # gets pos states 0,1,2
        vel   = state[3:6] #gets vel states 3,4,5
        quat  = quat_normalize(state[6:10]) # quation angleez 6,7,8,9
        omega = state[10:13] # angular speeds 10,11,12

        debug = {} # für spaeter ja

        # ---- 1. throttle: suicide-burn on TOTAL speed ----------------------
        alt = pos[2]
        vz  = vel[2] # negative when falling
        v_hor_mag = np.linalg.norm(vel[:2]) #betrag horizontale speed
        v_total = np.linalg.norm(vel) # total speed vector 3D
        a_max = F_MAX / MASS - G # gets max available vertical decel (in the real world this would not be this simple we are only getting away with this cuz simulation)
        speed_down = max(-vz, 0.0) # magic minus to invert speed value for our controller
        d_stop = speed_down**2 / (2.0 * a_max) if a_max > 0.1 else 1e9 # physik 9te klasse lässt grüßen (bremswegkalulation)

        # Vertical deceleration need
        if alt < 0.5:
            accel_z_des = -vz * 3.0 + G # if we are <0.5m from ground, switch to fallback P controller for smoother landing
        elif speed_down > 1.0: # 
            time_to_ground = alt / speed_down if speed_down > 0.5 else 1e6 # when we are still falling calc speed down
            accel_z_des = speed_down / max(time_to_ground, 0.5) + G # add gravity to compute destination speed
            accel_z_des = min(accel_z_des, F_MAX / MASS) # limit by true physics constraints fmax & mass
        else: # this pretty much only happens when we are veery slowly descending over an alt of 0.5m
            
            accel_z_des = G + 2.0 * (0.0 - vz) - 1.5  # slow vertical descent, bias toward descending, MAKE SURE WE DONT HOVER OH MY GOD

        # horizontal braking need, we muss kill v_hor in proportion to the remaining altitude, if not enough altitude left, brake harder 
        # junge das funktioniert nicht wir lenken unten am boden einfach übertrieben aus
        if v_hor_mag > 2.0:
            # time budget: use remaining altitude to estimate how long we have
            t_budget = alt / max(speed_down, 5.0) if speed_down > 1.0 else alt / 5.0
            t_budget = max(t_budget, 1.0)
            accel_hor_des = v_hor_mag / t_budget
            accel_hor_des = min(accel_hor_des, F_MAX / MASS * 0.5)  # max 50% for lateral movement, other should go to descent -> MAGIC NUMBAH TODO TODO TODO
        else:
            accel_hor_des = 0.0

        # total thrust: vector sum of vertical + horizontal needs
        accel_total = np.sqrt(accel_z_des**2 + accel_hor_des**2)
        throttle_nom = np.clip(accel_total * MASS / F_MAX, 0.0, 1.0)

        # ---- 2. desired attitude: retrograde → vertical -------------------
        #
        # Strategy: always point retrograde (= into the velocity vector)
        # which naturally kills ALL velocity components.  Once the rocket
        # is slow enough, transition to pure vertical so it descends
        # straight down and touches down wherever it happens to be.
        # No position-targeting, we land wherever we end up.

        v_hor  = np.linalg.norm(vel[:2]) # get speeds again (for my sanity)
        v_tot  = np.linalg.norm(vel)

        # retrograde direction 
        if v_tot > 5.0:
            retro_dir = -vel / v_tot
        else:
            retro_dir = np.array([0.0, 0.0, 1.0])  # straight up when slow

        # vertical (for final descent) dir
        vert_dir = np.array([0.0, 0.0, 1.0])

        # blending: retrograde while fast, vertical when slow
        # Transition when horizontal speed < 20 m/s and total < 50 m/s
        spd_frac = np.clip(max(v_hor / 20.0, v_tot / 50.0), 0.0, 1.0) # can go 1 or 0 depending on state, hard coded values TODO change
        #strategy:
        # spd_frac=1 -> fast -> retrograde; spd_frac=0 -> slow -> vertical
        desired_dir = spd_frac * retro_dir + (1.0 - spd_frac) * vert_dir
        # lineare Interpolation zwischen der rückwärtsrichtung und der senkrechten, basierend auf unserer geschwindigkeit
        # why are we doing this`TODO TODO`
        n2 = np.linalg.norm(desired_dir) # linearisieren soos
        if n2 > 1e-6:
            desired_dir /= n2
        else:
            desired_dir = np.array([0.0, 0.0, 1.0])

        q_des = align_body_z_to(desired_dir) # führungssollwert (destination) für den regler diggi

    #------------- 3. attitude to gimbal  (LQR or PD fallback) ---------------
        att_err = quat_error_vec(q_des, quat) # abweichung mit quaternionz, holt dann 3 winkel raus

        K = self._compute_lqr_gain(throttle_nom) # recompute every time, this also does the linearization
        # ok it worked, next steps:

        if K is not None: # if calculation from scipy was successful
            # LQR: u = −K x  produces corrective gimbal commands
            x_lqr = np.array([att_err[0], att_err[1], omega[0], omega[1]]) #fehler-zustandsvektor mit 4 dimensionen, lagefehler xy und rotationsraten in xy (z wird später geregelt)
            u_lqr = -K @ x_lqr # zustandsrückführung matrixprodukt mit matrix & fehler-zustandsvektor
            
            #safety gimbal limits
            gimbal_y = float(np.clip(u_lqr[0], -GIMBAL_MAX, GIMBAL_MAX))
            gimbal_z = float(np.clip(u_lqr[1], -GIMBAL_MAX, GIMBAL_MAX))
            
        else: # fallbackfallback TODO
            # PD fallback (same sign convention as ascent PID:
            # positive gimbal → negative moment → corrective)
            Kp = 0.5
            Kd = 0.25
            gimbal_y = float(np.clip(Kp * att_err[1] + Kd * omega[1],
                                     -GIMBAL_MAX, GIMBAL_MAX))
            gimbal_z = float(np.clip(Kp * att_err[0] + Kd * omega[0],
                                     -GIMBAL_MAX, GIMBAL_MAX))
            # magic numbers TODO -> ich mag das aber nicht kalibrieren.

        #roll dampening
        ail_z = float(np.clip(-800.0 * att_err[2] - 400.0 * omega[2],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE)) #aileron z rotation compensator (cheating)
        
        
        # für die ailerons: das ganze in LQR bauen war zu quarkig, deswegen mal wieder ein guter alter PD 
        # Pitch/yaw aileron assist grid fins provide direct torque to supplement the gimbal. 
        # magic number eingefügt (minus) weil richtung stimmt nicht
        # (positive err -> negative corrective torque).
        KP_AIL = 3000.0    # N·m per rad of error
        KD_AIL = 1500.0    # N·m·s per rad/s
        ail_x = float(np.clip(-KP_AIL * att_err[0] - KD_AIL * omega[0],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))
        ail_y = float(np.clip(-KP_AIL * att_err[1] - KD_AIL * omega[1],
                              -AILERON_MAX_TORQUE, AILERON_MAX_TORQUE))

        control = np.array([throttle_nom, gimbal_y, gimbal_z,
                            ail_x, ail_y, ail_z])

        debug['desired_dir'] = desired_dir
        debug['att_err']     = att_err
        debug['throttle']    = throttle_nom
        debug['d_stop']      = d_stop
        debug['alt']         = alt
        return control, debug
        # das könnte man auch schummeln nennen :) aber wir landen ja hauptsächlich mit der engine also ist es ok

    # ------------------------------------------------------------------
    def _compute_lqr_gain(self, throttle_nom): # compute gain and most importantly linearize
        """4-state LQR: [att_err_x, att_err_y, omega_x, omega_y]
        Controls: [gimbal_y, gimbal_z].
        """
        if not _HAS_SCIPY:
            return None # redundant
        try: # 
            T = max(throttle_nom * F_MAX, MASS * G * 0.3) # current throttle -> TODO magicn umber
            L = GIMBAL_TO_CG # "lever arm" that the gimbal has to the rockets CG
            # Linearised plant around vertical orientation.
            # mit Gemini:
            # From propulsion.py:
            #   F_body = T·[sin(gy), −sin(gz), cos(gy)cos(gz)]
            #   r_engine = [0, 0, −L]
            #   M = r × F:
            #     M_x = (−L)·(−T·sin(gz)) − 0 = … wait, let's do it properly:
            #     M = [0·Fz − (−L)·Fy,  (−L)·Fx − 0·Fz,  0]
            #       = [L·Fy,  −L·Fx,  0]
            #       = [L·(−T·sin(gz)),  −L·(T·sin(gy)),  0]
            #       = [−L·T·sin(gz),    −L·T·sin(gy),    0]
            #
            # SO kriegen wir das linearisiert für kleine winkel
            #   dω_x/dt = M_x / I_xx = −(L·T / I_xx)·gz = −g_x · gz
            #   dω_y/dt = M_y / I_yy = −(L·T / I_yy)·gy = −g_y · gy
            # => Sollte für jeden Schritt kein problem sein!
            # State: x = [e_x, e_y, ω_x, ω_y]
            # Control: u = [gy, gz]
            g_x = L * T / I_XX # linearized actuatoooorrrrrsssss
            g_y = L * T / I_YY

            A = np.array([
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ], dtype=float) # Systemmatrix TODO 

            # B: u = [gy, gz] → [dω_x/dt, dω_y/dt]
            #   dω_x/dt = −g_x · gz   →  B[2,1] = −g_x
            #   dω_y/dt = −g_y · gy   →  B[3,0] = −g_y
            B = np.array([
                [0,      0    ],
                [0,      0    ],
                [0,     -g_x  ],
                [-g_y,   0    ],
            ], dtype=float) # Eingangsmatrix -> passt

            Q = np.diag([40.0, 40.0, 4.0, 4.0]) # Kosten Gewichtungsmatrix für den zustand (lagefehler sind schlecht, rotationsgeschw ist okay)
            R = np.diag([1.0, 1.0]) # kosten gewichtungsmatrix für die aktoren TODO HIGHER! WE CANNOT SWING THE GIMBALS TOO MUCH

            P = solve_continuous_are(A, B, Q, R) # Lösung der Kontinuierlichen Algebraischen Riccati-Gleichung
            K = np.linalg.inv(R) @ B.T @ P # rückführ gain matrix aus VL
            self._K = K
            return K
        except np.linalg.LinAlgError:
            return self._K
        except ValueError:
            return self._K
        except Exception:
            return self._K        # return cached or None
