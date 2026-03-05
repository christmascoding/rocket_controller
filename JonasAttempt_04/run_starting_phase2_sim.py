#!/usr/bin/env python3
"""
run_starting_phase2_sim.py
==========================
Fast-iteration helper for Phase 2 / Phase 3 debugging.

Workflow
--------
1. First invocation (no snapshots.json): runs the **full** simulation
   to generate snapshots, then re-runs from the requested phase.
2. Subsequent invocations: loads the saved snapshot and simulates
   **only** from the chosen phase onward → much faster.

Usage:
    python run_starting_phase2_sim.py            # start from phase 2
    python run_starting_phase2_sim.py --phase 3  # start from phase 3
    python run_starting_phase2_sim.py --regen    # force full sim + new snapshots
    python run_starting_phase2_sim.py --headless # no visualisation
"""

import sys, os, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from src.simulation.simulator import Simulator
from src.simulation.phase_manager import PhaseManager
from src.models.trajectory import ReferenceTrajectory
from src.visualization.plotter import Plotter, plot_telemetry


SNAP_PATH = os.path.join(os.path.dirname(__file__), 'snapshots.json')


def ensure_snapshots(force: bool = False) -> dict:
    """Return snapshot dict, generating it from a full run if needed."""
    if not force and os.path.exists(SNAP_PATH):
        with open(SNAP_PATH, 'r') as f:
            snaps = json.load(f)
        if snaps:
            print(f"  [Snap] Loaded existing snapshots from {SNAP_PATH}")
            return snaps

    print("  [Snap] Running full simulation to generate snapshots …")
    sim = Simulator()
    sim.run()
    sim.pm.save_snapshots(SNAP_PATH)
    print(f"  [Snap] Saved → {SNAP_PATH}")
    return sim.pm.snapshots


def main():
    parser = argparse.ArgumentParser(description="Run sim from a saved phase.")
    parser.add_argument('--phase', default='2', choices=['2', '3'],
                        help="Phase to start from (default: 2)")
    parser.add_argument('--regen', action='store_true',
                        help="Force a full sim to regenerate snapshots")
    parser.add_argument('--headless', action='store_true',
                        help="No visualisation, just print summary")
    args = parser.parse_args()

    print("═" * 60)
    print("  TTHopper — Fast Phase-Restart Simulation")
    print("═" * 60)

    # 1. Ensure we have snapshots
    snaps = ensure_snapshots(force=args.regen)

    target_phase = args.phase
    if target_phase not in snaps:
        print(f"  [Error] No snapshot for phase '{target_phase}'.")
        print(f"           Available: {list(snaps.keys())}")
        print("           Re-run with --regen to create fresh snapshots.")
        sys.exit(1)

    snap = snaps[target_phase]
    t0    = snap['t']
    state0 = np.array(snap['state'])

    print(f"\n  Resuming from phase {target_phase} at t = {t0:.2f} s")
    print(f"    pos  = [{state0[0]:.1f}, {state0[1]:.1f}, {state0[2]:.1f}] m")
    print(f"    vel  = [{state0[3]:.1f}, {state0[4]:.1f}, {state0[5]:.1f}] m/s")
    print()

    # 2. Run from the snapshot
    sim = Simulator()
    logger = sim.run_from_phase(target_phase, t0, state0)

    # 3. Export
    out_path = os.path.join(os.path.dirname(__file__),
                            f'sim_results_from_phase{target_phase}.json')
    logger.save_json(out_path)

    if args.headless:
        print("  [Headless] Done.")
        return

    # 4. Visualisation (same as run_demo.py)
    data = logger.to_dict()
    ref = ReferenceTrajectory()
    ref_xyz = ref.sample(500)

    print("\n  Opening 3-D animation …")
    plotter = Plotter(data, ref_traj_xyz=ref_xyz)
    plotter.show()

    print("  Opening telemetry plots …")
    plot_telemetry(data)


if __name__ == '__main__':
    main()
