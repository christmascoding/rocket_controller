import numpy as np

# Physical constants
G = 9.81
RHO = 1.225

# Rocket geometry and mass properties
LENGTH = 5.825  # m
DIAMETER = 0.7  # m
RADIUS = DIAMETER / 2.0
MASS = 10_000.0  # kg
MAX_THRUST = 300_000.0  # N

# CG and CP
CG_FROM_GIMBAL = 2.8  # m (CG above engine gimbal point)
CP_OFFSET_ASCENT = 0.6  # m behind CG along -body Z
CP_OFFSET_PHASE2 = 0.8  # m ahead of CG along +body Z (grid fins shift)

# Moments of inertia (cylinder, body Z along length)
I_ZZ = 0.5 * MASS * RADIUS ** 2
I_XX = (1.0 / 12.0) * MASS * (3 * RADIUS ** 2 + LENGTH ** 2)
I_YY = I_XX
INERTIA = np.diag([I_XX, I_YY, I_ZZ])

# Aerodynamics
S_REF = np.pi * RADIUS ** 2
CL_ALPHA = 3.5  # 1/rad
CD0 = 0.3
CD_ALPHA = 1.2  # quadratic term

# Control limits
GIMBAL_MAX_DEG = 15.0
GIMBAL_MAX = np.deg2rad(GIMBAL_MAX_DEG)
GIMBAL_MAX_PHASE3_DEG = 10.0
GIMBAL_MAX_PHASE3 = np.deg2rad(GIMBAL_MAX_PHASE3_DEG)
THROTTLE_MIN_PHASE3 = 0.3
LAUNCH_GIMBAL_MAX_DEG = 3.0
LAUNCH_GIMBAL_MAX = np.deg2rad(LAUNCH_GIMBAL_MAX_DEG)

# Simple aerodynamic control authority for descent
AERO_TORQUE_MAX = 35_000.0  # N*m

# Controller gains
Kp_pos = np.array([4.5, 4.5, 0.90])
Kd_pos = np.array([6.5, 6.5, 1.00])

Kp_att = 300_000.0
Kd_att = 40_000.0

# Phase 1C specific attitude control gains (aggressive gimbal for flip maneuver)
Kp_att_phase1c = 500_000.0  # Higher proportional gain for immediate gimbal response
Kd_att_phase1c = 140_000.0  # Higher damping to prevent overshoot during 180° flip

Kp_z = 0.08
Kd_z = 0.6
Kp_xy_phase3 = 0.015

# Middle-loop tilt amplification
MIDDLE_TILT_GAIN = 2.5

# Phase 2 attitude/velocity damping
PHASE2_TILT_MAX_DEG = 10.0
PHASE2_TILT_MAX = np.deg2rad(PHASE2_TILT_MAX_DEG)
PHASE2_VEL_DAMP = 0.04

# Phase 2 flip controller (smooth 180° rotation)
PHASE2_FLIP_Kp = 8_000.0
PHASE2_FLIP_Kd = 25_000.0  # High derivative for braking

# Phase 2 angular rate damping (anti-spiral)
PHASE2_RATE_DAMP = 50_000.0
PHASE2_AERO_DAMP = 15_000.0  # Aerodynamic damping torque
PHASE2_AERO_DAMP_SCALE = 0.15  # Reduce damping to allow sideways slip
PHASE2_SIDE_TORQUE = 4_000.0  # N·m constant roll bias for lateral drift

# Phase 1c: Powered Flip at apogee
PHASE1C_THROTTLE = 0.75  # 75% thrust for better control authority during flip
PHASE1C_FLIP_KP = 1.5  # Proportional gain: [rad/s angular velocity / rad angle error]
PHASE1C_FLIP_KD = 50000.0  # Derivative gain: [N⋅m torque / rad/s angular velocity error]
PHASE1C_FLIP_SUCCESS_THETA_ERR = np.deg2rad(2.0)  # 2° tolerance - much stricter alignment
PHASE1C_FLIP_SUCCESS_RATE = 0.05  # rad/s angular rate threshold - much stricter stability
PHASE1C_FLIP_TIMEOUT = 60.0  # seconds - allow up to 60s for flip

# Phase 3 slew rate limiting (soft-start)
PHASE3_GIMBAL_SLEW_LIMIT = 0.7  # rad/s (2°/frame @ dt=0.05s) - prevents bang-bang
PHASE3_RATE_PRIORITY_TIME = 0.2  # seconds to prioritize rate damping only
PHASE3_SOFT_ENGAGEMENT_TIME = 0.5  # seconds for gradual gain ramp-up
PHASE3_ROLL_DAMPING = 5_000.0  # Artificial roll damping (simulates RCS)
PHASE3_IGNITION_SAFETY_MARGIN = 0.98  # Trigger at d_stop >= 0.98*Z (2% margin - precision burn)
PHASE3_ATTITUDE_ONLY_ALT = 50.0  # m - below this, allow horizontal corrections
PHASE3_RETRO_TILT_GAIN = 1.0  # scale for retrograde tilt command (1.0 ~= v/g)
PHASE3_RETRO_TILT_MAX_DEG = 20.0  # max tilt for retrograde pointing
PHASE3_RETRO_TILT_MAX = np.deg2rad(PHASE3_RETRO_TILT_MAX_DEG)

# Fin/RCS control gains (reduced)
Kp_fin = 1_500.0
Kd_fin = 400.0

# Launch smoothing (limit initial tracking error)
LAUNCH_POS_ERR_MAX = 500.0  # m
LAUNCH_VEL_ERR_MAX = 50.0  # m/s
LAUNCH_T_LIMIT = 20.0  # s

# Pure Pursuit guidance
PURE_PURSUIT_LOOKAHEAD = 2000.0  # m
MAX_PITCH_FROM_VERTICAL_DEG = 60.0
MAX_PITCH_FROM_VERTICAL = np.deg2rad(MAX_PITCH_FROM_VERTICAL_DEG)
PURE_PURSUIT_END_THRESHOLD = 0.95  # disable PP when within last 5% of path

# LQR weights (phase 3) - state: [vx, vy, theta, psi, q, r] (NO ROLL)
# CRITICAL: Prioritize angular rates >> angles >> velocity for stability
LQR_Q = np.diag([8.0, 8.0, 25.0, 25.0, 50.0, 50.0])  # vel, angles, rates (RATES FIRST!)
LQR_R = np.diag([50.0, 50.0])  # gimbal effort (INCREASED 50x - forces tiny, precise movements)

# Simulation settings
SIM_DT = 0.05  # s
SIM_T_FINAL = 140.0  # s
SIM_T_MAX = 800.0  # s

# Trajectory settings (quarter ellipse)
TRAJ_X_REF = 70_000.0  # m
TRAJ_Z_REF = 80_000.0  # m
TRAJ_T_FINAL = 140.0  # s (MECO)
MECO_ALT_MAX = 75_000.0  # m (dynamic cutoff)
MECO_TIME_MAX = 70.0  # s (dynamic cutoff)
