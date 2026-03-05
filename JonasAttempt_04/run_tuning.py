#!/usr/bin/env python3
"""
run_tuning.py — Example headless tuning script.

Demonstrates the snapshot-based optimisation workflow:
  1. Load a phase snapshot
  2. Build a cost function
  3. Run scipy optimisation over controller parameters
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
from src.tuning.optimizer import optimize_phase
from src.controllers.landing_lqr import LandingLQR


def landing_cost(history):
    """Cost function for Phase 3 (landing).

    J = w1 · |p_final − p_target|² + w2 · |v_final|² + w3 · ∫|u|² dt
    """
    final = history[-1]
    pos = final['state'][:3]
    vel = final['state'][3:6]

    target = np.array([0.0, 0.0, 0.0])
    pos_err = np.linalg.norm(pos - target)
    vel_err = np.linalg.norm(vel)

    # control effort
    effort = sum(np.linalg.norm(h['control'])**2 for h in history)
    effort /= len(history)

    w1, w2, w3 = 10.0, 50.0, 0.01
    return w1 * pos_err**2 + w2 * vel_err**2 + w3 * effort


def lqr_factory(params):
    """Create a LandingLQR instance (gains are in config; this is a stub
    that can be extended to inject tunable parameters)."""
    ctrl = LandingLQR()
    return ctrl


def main():
    snap_path = os.path.join(os.path.dirname(__file__), 'snapshots.json')
    if not os.path.exists(snap_path):
        print("No snapshots.json found.  Run 'python run_demo.py --headless' first.")
        return

    print("Starting headless optimisation for Phase 3 …")
    # For demo purposes this uses trivial bounds — extend as needed.
    result = optimize_phase(
        snapshot_path=snap_path,
        controller_factory=lqr_factory,
        cost_fn=landing_cost,
        param_bounds=[(0.1, 50.0)] * 2,   # placeholder tunable params
        method='Nelder-Mead',
        maxiter=50,
    )
    print(f"  Best cost:   {result['best_cost']:.4f}")
    print(f"  Best params: {result['best_params']}")


if __name__ == '__main__':
    main()
