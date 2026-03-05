"""
Configuration for TTHopper 6-DOF Rocket Simulation.

All physical constants, rocket parameters, mission parameters,
phase-transition thresholds, and controller gains live here.

Coordinate conventions
----------------------
World frame (inertial, ENU-like):
    X = downrange (east), Y = crossrange (north), Z = up (altitude)
Body frame (fixed to rocket):
    z_B = longitudinal axis from engine → nose (up when vertical)
    x_B, y_B = lateral axes completing a right-hand system
Quaternion convention:
    Scalar-first  q = [w, x, y, z],  rotates body → world.
    At launch (vertical):  q = [1, 0, 0, 0]  (identity).
"""

import numpy as np

# ═══════════════════════════════════════════════════════════════════════════
# 1.  ROCKET PARAMETERS  (TTHopper)
# ═══════════════════════════════════════════════════════════════════════════
MASS            = 10_000.0          # kg  (total, including propellant)
LENGTH          = 5.825             # m
CG_FROM_BOTTOM  = 2.913             # m   (height of CG above engine)
DIAMETER        = 0.7               # m
RADIUS          = DIAMETER / 2.0

# Moments of inertia — solid-cylinder approximation
I_XX = MASS / 12.0 * (3.0 * RADIUS**2 + LENGTH**2)   # lateral  ≈ 28 583
I_YY = I_XX                                            # symmetric
I_ZZ = MASS / 2.0 * RADIUS**2                          # roll     ≈ 612
INERTIA     = np.diag([I_XX, I_YY, I_ZZ])
INERTIA_INV = np.linalg.inv(INERTIA)

# ═══════════════════════════════════════════════════════════════════════════
# 2.  PROPULSION
# ═══════════════════════════════════════════════════════════════════════════
F_MAX           = 200_000.0         # N   (configurable max thrust)
GIMBAL_MAX      = np.radians(7.5)   # rad (max nozzle deflection)
GIMBAL_RATE_MAX = np.radians(20.0)  # rad/s  (slew-rate limit)
GIMBAL_TO_CG    = CG_FROM_BOTTOM    # m   (lever arm: engine → CG)

# ═══════════════════════════════════════════════════════════════════════════
# 3.  AERODYNAMICS
# ═══════════════════════════════════════════════════════════════════════════
S_REF       = np.pi * RADIUS**2     # m²  reference area
CL_ALPHA    = 3.5                   # lift-curve slope  (/rad)
CD_0        = 0.3                   # zero-AoA drag coefficient
CD_ALPHA    = 1.2                   # AoA drag increment  (/rad²)

# Centre-of-pressure offset from CG along body z  (+ → toward nose)
#   Ascent (no grid fins):   CP slightly ahead of CG → unstable → needs TVC
#   Descent with grid fins:  CP behind CG → statically stable
CP_OFFSET_ASCENT   =  0.5          # m
CP_OFFSET_DESCENT  = -1.5          # m

# Maximum aerodynamic control-surface torque (grid fins + RCS equiv.)
AILERON_MAX_TORQUE = 5_000.0       # N·m

# ═══════════════════════════════════════════════════════════════════════════
# 4.  ATMOSPHERE / GRAVITY
# ═══════════════════════════════════════════════════════════════════════════
RHO_0   = 1.225                     # kg/m³  sea-level density
H_SCALE = 8_500.0                   # m      scale height
G       = 9.81                      # m/s²

# ═══════════════════════════════════════════════════════════════════════════
# 5.  MISSION  &  TRAJECTORY
# ═══════════════════════════════════════════════════════════════════════════
MECO_TIME       = 80.0              # s   main-engine cut-off time
T_COAST_END     = 140.0             # s   approx time to apogee (ref traj)
X_DOWNRANGE     = 70_000.0          # m
Z_APOGEE        = 80_000.0          # m
LANDING_TARGET  = np.array([0.0, 0.0, 0.0])   # return-to-launch-site

# ═══════════════════════════════════════════════════════════════════════════
# 6.  PHASE-TRANSITION CRITERIA
# ═══════════════════════════════════════════════════════════════════════════
FLIP_ANGLE_THRESHOLD = np.radians(10.0)   # within 10° of retrograde → done
FLIP_TIMEOUT         = 60.0                # s max for flip manoeuvre
FLIP_THROTTLE        = 0.25               # nominal throttle during flip

SUICIDE_BURN_MARGIN  = 1.0000001               #  safety factor on stopping dist.
LANDING_ALT          = 1.0                # m   touchdown altitude
LANDING_VEL          = 2.0                # m/s max touchdown speed

# ═══════════════════════════════════════════════════════════════════════════
# 7.  SIMULATION
# ═══════════════════════════════════════════════════════════════════════════
DT      = 0.02      # s   integration time-step
T_MAX   = 600.0     # s   hard stop
LOG_DECIMATION = 5   # store every N-th step → effective 10 Hz at DT=0.02

# ═══════════════════════════════════════════════════════════════════════════
# 8.  CONTROLLER GAINS  (initial / default — tuning overwrites via JSON)
# ═══════════════════════════════════════════════════════════════════════════

# --- Phase 1a / 1b : cascaded PD ------------------------------------------
PID_KP_POS  = np.array([0.3,  0.3,  0.5])    # position → velocity
PID_KD_VEL  = np.array([1.5,  1.5,  2.5])    # velocity → acceleration

# Inner-loop attitude gains — derived from plant dynamics.
#   Plant gain: G = L·F_max / I ≈ 20.4 rad/s² per rad of gimbal.
#   Target bandwidth: ωn ≈ 3 rad/s, damping: ζ ≈ 0.85.
#   KP = ωn²/G ≈ 0.45,  KD = 2·ζ·ωn/G ≈ 0.25.
# Saturates linearly at ~17° error (GIMBAL_MAX/KP).
PID_KP_ATT  = 0.5                             # attitude error → gimbal
PID_KD_ATT  = 0.25                            # angular-rate damping

# Roll-axis gains (direct torque, no gimbal geometry inversion).
# Plant gain: G_roll = 1/I_ZZ ≈ 0.00163.  Target ωn_roll ≈ 3 rad/s.
#   KP_roll = ωn² · I_ZZ ≈ 5500,  KD_roll = 2·ζ·ωn · I_ZZ ≈ 3300.
ROLL_KP     = 5500.0                           # N·m per rad
ROLL_KD     = 3300.0                           # N·m·s per rad

# --- Phase 1c : MPC flip --------------------------------------------------
MPC_HORIZON   = 20          # steps
MPC_DT        = 0.1         # s per prediction step
MPC_W_ANGLE   = 100.0       # terminal angle cost
MPC_W_RATE    = 20.0        # terminal rate cost
MPC_W_CONTROL = 1.0         # running control cost

# --- Phase 2 : sliding-mode / gain-scheduled ------------------------------
SMC_LAMBDA  = 2.0           # sliding surface slope
SMC_K       = 5.0           # switching gain
SMC_PHI     = 0.1           # boundary-layer width (chattering suppression)

# --- Phase 3 : LQR landing ------------------------------------------------
#  Q weights on [x,y,z, vx,vy,vz, phi,theta, p,q]  (10-state reduced model)
LQR_Q_DIAG  = [5, 5, 100,  1, 1, 20,  40, 40,  4, 4]
LQR_R_DIAG  = [0.1, 1.0, 1.0]   # throttle, gimbal_y, gimbal_z
