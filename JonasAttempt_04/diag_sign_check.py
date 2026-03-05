"""Diagnostic script: trace the LQR sign chain end-to-end."""
import sys, os; sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
from src.config import *
from src.math_utils import *
from src.physics.propulsion import compute_thrust
from scipy.linalg import solve_continuous_are

# Scenario: during landing, rocket falling DOWN, body z ~ world +z (up)
# Desired = identity (body z = world z). Actual: 5 deg tilt about +y.
theta = np.radians(5)
q_actual = np.array([np.cos(theta/2), 0, np.sin(theta/2), 0])
q_desired = np.array([1.0, 0.0, 0.0, 0.0])

att_err = quat_error_vec(q_desired, q_actual)
print(f"att_err = {att_err}")
print(f"  att_err[0]={att_err[0]:.4f}  att_err[1]={att_err[1]:.4f}")

# Compute LQR K at throttle=0.8
throttle = 0.8
T = throttle * F_MAX
L = GIMBAL_TO_CG
b_x = L * T / I_XX
b_y = L * T / I_YY
print(f"b_x={b_x:.4f}, b_y={b_y:.4f}")

A = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]], dtype=float)
# CORRECT B: matches actual propulsion signs
#   M_x = +L*T*gz  →  dω_x/dt = +b_x * gz   →  B[2,1] = +b_x
#   M_y = -L*T*gy  →  dω_y/dt = -b_y * gy   →  B[3,0] = -b_y
B = np.array([[0,0],[0,0],[0,b_x],[-b_y,0]], dtype=float)
Q = np.diag([40.0, 40.0, 4.0, 4.0])
R = np.diag([1.0, 1.0])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print(f"K =\n{K}")

# Controller: u = -K @ x_lqr
omega = np.zeros(3)
x_lqr = np.array([att_err[0], att_err[1], omega[0], omega[1]])
u_lqr = -K @ x_lqr
print(f"u_lqr = {u_lqr}")
print(f"  gimbal_y = {np.degrees(u_lqr[0]):.3f} deg")
print(f"  gimbal_z = {np.degrees(u_lqr[1]):.3f} deg")

# Propulsion moment
gy = float(np.clip(u_lqr[0], -GIMBAL_MAX, GIMBAL_MAX))
gz = float(np.clip(u_lqr[1], -GIMBAL_MAX, GIMBAL_MAX))
F_body, M_body = compute_thrust(throttle, gy, gz)
print(f"M_body = {M_body}")

alpha = INERTIA_INV @ M_body
print(f"alpha  = {alpha}")
print()
print("Error is 5 deg about +y. To correct, need alpha_y < 0.")
if alpha[1] < 0:
    print(">> RESULT: alpha_y < 0 => CORRECT (negative feedback)")
else:
    print(">> RESULT: alpha_y > 0 => WRONG (positive feedback / spin-up)")

# Also check: what does the PD fallback path produce?
print("\n--- PD fallback check ---")
Kp = 0.5; Kd = 0.25
gy_pd = float(np.clip(Kp * att_err[1] + Kd * omega[1], -GIMBAL_MAX, GIMBAL_MAX))
gz_pd = float(np.clip(Kp * att_err[0] + Kd * omega[0], -GIMBAL_MAX, GIMBAL_MAX))
print(f"PD gimbal_y = {np.degrees(gy_pd):.3f} deg, gimbal_z = {np.degrees(gz_pd):.3f} deg")
F_pd, M_pd = compute_thrust(throttle, gy_pd, gz_pd)
alpha_pd = INERTIA_INV @ M_pd
print(f"PD alpha = {alpha_pd}")
if alpha_pd[1] < 0:
    print(">> PD: alpha_y < 0 => CORRECT")
else:
    print(">> PD: alpha_y > 0 => WRONG (positive feedback)")

# Now check: what SIGN does the propulsion actually produce?
print("\n--- Raw propulsion sign check ---")
print("Small positive gimbal_y:")
_, M_pos_gy = compute_thrust(0.8, 0.01, 0.0)
print(f"  M = {M_pos_gy}  => M_y = {M_pos_gy[1]:.2f}")
print("Small positive gimbal_z:")
_, M_pos_gz = compute_thrust(0.8, 0.0, 0.01)
print(f"  M = {M_pos_gz}  => M_x = {M_pos_gz[0]:.2f}")
