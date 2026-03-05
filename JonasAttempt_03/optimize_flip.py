import numpy as np
import json
from scipy.optimize import minimize
from src.controllers.mpc_flip import (
    rocket_flip_dynamics,
    mpc_flip_cost,
    load_flip_scenario
)

# Optimization target: minimize final attitude error and rate, ensure stability

def evaluate_flip(params, scenario, rocket, N=20, dt=0.1, target=[np.pi, 0.0]):
    """
    Run MPC flip with given params and scenario, return performance metric.
    params: [w_theta, w_psi, w_q, w_r, w_u]
    scenario: dict with state0
    rocket: rocket object
    N: horizon steps
    dt: timestep
    target: desired attitude
    """
    w_theta, w_psi, w_q, w_r, w_u = params
    state0 = scenario['state0']
    u0 = np.zeros((N, 2)).flatten()
    bounds = [(-np.deg2rad(15), np.deg2rad(15))] * (N * 2)

    def custom_cost(u_flat, state0, rocket, N, dt, target):
        u = u_flat.reshape(N, 2)
        state = np.array(state0)
        cost = 0.0
        for k in range(N):
            state = rocket_flip_dynamics(state, u[k], rocket, dt)
            theta_err = state[0] - target[0]
            psi_err = state[1] - target[1]
            q = state[2]
            r = state[3]
            cost += w_theta * theta_err**2 + w_psi * psi_err**2 + w_q * q**2 + w_r * r**2
            cost += w_u * (u[k][0]**2 + u[k][1]**2)
        # Penalize instability: if rates are too high at end, add large penalty
        if abs(state[2]) > 1.0 or abs(state[3]) > 1.0:
            cost += 1000.0
        return cost

    res = minimize(custom_cost, u0, args=(state0, rocket, N, dt, target), bounds=bounds)
    u_opt = res.x.reshape(N, 2)
    # Simulate the flip with optimal controls
    state = np.array(state0)
    for k in range(N):
        state = rocket_flip_dynamics(state, u_opt[k], rocket, dt)
    # Final error and rate
    final_theta_err = abs(state[0] - target[0])
    final_psi_err = abs(state[1] - target[1])
    final_q = abs(state[2])
    final_r = abs(state[3])
    # Performance metric: sum of final errors and rates
    return final_theta_err + final_psi_err + final_q + final_r

# Automated optimization loop

def optimize_flip_scenario(scenario_path, rocket, N=20, dt=0.1, target=[np.pi, 0.0]):
    from scipy.optimize import differential_evolution
    scenario = load_flip_scenario(scenario_path)
    bounds = [(1, 100), (1, 100), (0.1, 10), (0.1, 10), (0.01, 1)]  # weights for theta, psi, q, r, u
    def objective(params):
        return evaluate_flip(params, scenario, rocket, N, dt, target)
    result = differential_evolution(objective, bounds, maxiter=20, disp=True)
    print("Best params:", result.x)
    print("Best performance:", result.fun)
    return result.x

if __name__ == "__main__":
    from src.models.rocket import Rocket
    import os
    scenario_path = os.path.join(os.path.dirname(__file__), "flip_scenario.json")
    rocket = Rocket()
    best_params = optimize_flip_scenario(scenario_path, rocket)
    print("Optimized MPC weights:", best_params)
