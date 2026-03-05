"""Quick verification that both LQR axes produce corrective torque."""
import sys, numpy as np
sys.path.insert(0, '.')
from src.config import *
from src.physics.propulsion import compute_thrust
from scipy.linalg import solve_continuous_are

T = 0.7 * F_MAX   # typical landing throttle
L = GIMBAL_TO_CG
g_x = L * T / I_XX
g_y = L * T / I_YY

A = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]], dtype=float)
B = np.array([[0,0],[0,0],[0,-g_x],[-g_y,0]], dtype=float)
Q = np.diag([40,40,4,4])
R = np.diag([1,1])

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print("K =", K)

# --- X axis tilt (+5 deg about x) --- needs alpha_x < 0
ex = 0.0872
u = -K @ np.array([ex, 0, 0, 0])
gy, gz = np.clip(u, -GIMBAL_MAX, GIMBAL_MAX)
F, M = compute_thrust(T, gy, gz)
ax = M[0] / I_XX
print(f"X-tilt: gy={gy:.4f} gz={gz:.4f} Mx={M[0]:.1f} ax={ax:.4f} {'OK' if ax<0 else 'WRONG'}")

# --- Y axis tilt (+5 deg about y) --- needs alpha_y < 0
ey = 0.0872
u2 = -K @ np.array([0, ey, 0, 0])
gy2, gz2 = np.clip(u2, -GIMBAL_MAX, GIMBAL_MAX)
F2, M2 = compute_thrust(T, gy2, gz2)
ay = M2[1] / I_YY
print(f"Y-tilt: gy={gy2:.4f} gz={gz2:.4f} My={M2[1]:.1f} ay={ay:.4f} {'OK' if ay<0 else 'WRONG'}")
