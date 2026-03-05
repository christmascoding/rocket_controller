import json

def save_flip_scenario(filepath, scenario):
    """
    Save the flip scenario (dict) to a JSON file.
    """
    # Convert numpy arrays to lists for JSON serialization
    scenario_serializable = {}
    for k, v in scenario.items():
        if isinstance(v, np.ndarray):
            scenario_serializable[k] = v.tolist()
        else:
            scenario_serializable[k] = v
    with open(filepath, 'w') as f:
        json.dump(scenario_serializable, f, indent=2)

def load_flip_scenario(filepath):
    """
    Load the flip scenario (dict) from a JSON file.
    """
    with open(filepath, 'r') as f:
        scenario = json.load(f)
    # Convert lists back to numpy arrays where appropriate
    for k, v in scenario.items():
        if isinstance(v, list):
            scenario[k] = np.array(v)
    return scenario
import numpy as np
from scipy.optimize import minimize

# Minimal state: [theta, psi, q, r]
# Control: [delta_y, delta_z]

def rocket_flip_dynamics(state, control, rocket, dt):
    """
    Nonlinear step for rotational dynamics (Euler integration).
    state: [theta, psi, q, r]
    control: [delta_y, delta_z]
    rocket: rocket object with inertia, thrust, cg_from_gimbal
    dt: timestep
    """
    theta, psi, q, r = state
    delta_y, delta_z = control

    # Parameters
    Iyy = rocket.inertia[1, 1]
    Ixx = rocket.inertia[0, 0]
    thrust = rocket.max_thrust * 0.75  # Use fixed throttle for now
    L = rocket.cg_from_gimbal

    # Torques from gimbal
    tau_pitch = thrust * delta_y * L
    tau_yaw = thrust * delta_z * L

    # Dynamics
    q_dot = tau_pitch / Iyy
    r_dot = tau_yaw / Ixx

    # Euler integration
    theta_next = theta + q * dt
    psi_next = psi + r * dt
    q_next = q + q_dot * dt
    r_next = r + r_dot * dt

    return np.array([theta_next, psi_next, q_next, r_next])


def mpc_flip_cost(u_flat, state0, rocket, N, dt, target):
    """
    Cost function for MPC flip.
    u_flat: flattened control sequence [delta_y0, delta_z0, ..., delta_yN-1, delta_zN-1]
    state0: initial state
    rocket: rocket object
    N: horizon steps
    dt: timestep
    target: [theta_des, psi_des]
    """
    u = u_flat.reshape(N, 2)
    state = np.array(state0)
    cost = 0.0
    for k in range(N):
        state = rocket_flip_dynamics(state, u[k], rocket, dt)
        # Penalize attitude error and rates
        theta_err = state[0] - target[0]
        psi_err = state[1] - target[1]
        q = state[2]
        r = state[3]
        cost += 10.0 * theta_err**2 + 10.0 * psi_err**2 + 1.0 * q**2 + 1.0 * r**2
        # Penalize control effort
        cost += 0.1 * (u[k][0]**2 + u[k][1]**2)
    return cost

# Example usage (to be integrated into controller):
# N = 20
# dt = 0.1
# state0 = [theta, psi, q, r]
# target = [np.pi, 0.0]  # Flip to pi pitch, 0 yaw
# u0 = np.zeros((N, 2)).flatten()
# bounds = [(-np.deg2rad(15), np.deg2rad(15))] * (N * 2)
# res = minimize(mpc_flip_cost, u0, args=(state0, rocket, N, dt, target), bounds=bounds)
# u_opt = res.x.reshape(N, 2)
# Apply u_opt[0] as the next control input
