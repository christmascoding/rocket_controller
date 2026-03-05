#!/usr/bin/env python3
"""
run_demo.py — Run the full TTHopper 6-DOF simulation and show results.

Usage:
    python run_demo.py              # simulate + animate + telemetry
    python run_demo.py --headless   # simulate only, export JSON
"""

import sys, os, json

# Ensure package root is importable
sys.path.insert(0, os.path.dirname(__file__))

from src.simulation.simulator import Simulator
from src.models.trajectory import ReferenceTrajectory
from src.visualization.plotter import Plotter, plot_telemetry


def main():
    headless = '--headless' in sys.argv

    print("═" * 60)
    print("  TTHopper 6-DOF Rocket Simulation — JonasAttempt_04")
    print("═" * 60)

    # --- simulation ---
    sim = Simulator()
    logger = sim.run()

    # --- export JSON ---
    out_path = os.path.join(os.path.dirname(__file__), 'sim_results.json')
    logger.save_json(out_path)

    # --- save snapshots ---
    snap_path = os.path.join(os.path.dirname(__file__), 'snapshots.json')
    sim.pm.save_snapshots(snap_path)

    if headless:
        print("  [Headless] Done.  Results → sim_results.json")
        return

    # --- visualisation ---
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
