"""
Headless optimiser — runs a single phase many times and uses
scipy.optimize to minimise a user-supplied cost function over
the controller parameters.

Usage
-----
    from src.tuning.optimizer import optimize_phase
    best = optimize_phase(
        snapshot_path = 'snapshots/phase_3.json',
        cost_fn       = my_landing_cost,
        param_bounds  = [(0.1, 100), ...],
        method        = 'Nelder-Mead',
    )
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution

from src.config import DT
from src.math_utils import quat_normalize
from src.physics.dynamics import state_derivative
from src.tuning.snapshot import load_snapshot


def run_phase_headless(state0: np.ndarray, controller, phase: str,
                       dt: float = DT, t_max: float = 300.0,
                       log_every: int = 1) -> list[dict]:
    """Forward-integrate a single phase without GUI.

    Returns a list of dicts with keys 't', 'state', 'control'.
    """
    state = state0.copy()
    history = []
    t = 0.0
    step = 0

    while t < t_max:
        info = {'phase': phase, 'phase_time': t}
        control, _ = controller.compute(t, state, info)

        if step % log_every == 0:
            history.append({'t': t, 'state': state.copy(),
                            'control': control.copy()})

        # RK4
        k1 = state_derivative(state,               control, phase)
        k2 = state_derivative(state + 0.5*dt*k1,   control, phase)
        k3 = state_derivative(state + 0.5*dt*k2,   control, phase)
        k4 = state_derivative(state + dt*k3,        control, phase)
        state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        state[6:10] = quat_normalize(state[6:10])

        t += dt
        step += 1

        # ground clamp
        if state[2] < 0.0:
            state[2] = 0.0
            state[5] = max(state[5], 0.0)
            history.append({'t': t, 'state': state.copy(),
                            'control': control.copy()})
            break

    return history


def optimize_phase(snapshot_path: str,
                   controller_factory,
                   cost_fn,
                   param_bounds: list[tuple[float, float]],
                   method: str = 'Nelder-Mead',
                   maxiter: int = 200,
                   **kwargs) -> dict:
    """
    Parameters
    ----------
    snapshot_path      : path to JSON snapshot
    controller_factory : callable(params) → controller instance
    cost_fn            : callable(history) → float  (lower is better)
    param_bounds       : list of (lo, hi) for each tunable parameter
    method             : 'Nelder-Mead' | 'differential_evolution' | …

    Returns
    -------
    dict with 'best_params', 'best_cost', 'result'.
    """
    snap = load_snapshot(snapshot_path)
    state0 = snap['state']
    phase  = snap['phase']

    def objective(params):
        ctrl = controller_factory(params)
        hist = run_phase_headless(state0, ctrl, phase)
        return cost_fn(hist)

    if method == 'differential_evolution':
        result = differential_evolution(objective, param_bounds,
                                        maxiter=maxiter, seed=42,
                                        **kwargs)
    else:
        x0 = np.array([(lo + hi) / 2 for lo, hi in param_bounds])
        result = minimize(objective, x0, method=method,
                          options={'maxiter': maxiter}, **kwargs)

    return {
        'best_params': result.x.tolist(),
        'best_cost': float(result.fun),
        'result': result,
    }
