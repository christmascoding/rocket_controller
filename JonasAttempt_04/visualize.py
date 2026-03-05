#!/usr/bin/env python3
"""
visualize.py — Offline visualisation from an exported JSON file.

Usage:
    python visualize.py                       # loads sim_results.json
    python visualize.py path/to/results.json
"""

import sys, os, json
sys.path.insert(0, os.path.dirname(__file__))

from src.models.trajectory import ReferenceTrajectory
from src.visualization.plotter import Plotter, plot_telemetry


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'sim_results.json'
    if not os.path.exists(path):
        print(f"File not found: {path}")
        return

    with open(path) as f:
        data = json.load(f)

    ref = ReferenceTrajectory()
    ref_xyz = ref.sample(500)

    print(f"Loaded {len(data['time'])} samples from {path}")

    plotter = Plotter(data, ref_traj_xyz=ref_xyz)
    plotter.show()

    plot_telemetry(data)


if __name__ == '__main__':
    main()
