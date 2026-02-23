#!/usr/bin/env python3
"""
Check what the target trajectory actually is.
"""

import sys
sys.path.insert(0, '.')

from src.models.trajectory import CurvedAscentPath
import numpy as np

trajectory = CurvedAscentPath(
    peak_altitude=60000.0,
    horizontal_distance=40000.0
)

print("="*80)
print("TARGET TRAJECTORY INSPECTION")
print("="*80)
print()

print(f"Total time: {trajectory.total_time:.2f}s")
print()
print(f"{'Time(s)':>10} | {'X(m)':>12} | {'Z(m)':>12} | {'VX(m/s)':>12} | {'VZ(m/s)':>12}")
print("-"*80)

for t in [0, 10, 20, 30, 50, 60, 80, 100, 110.6, 120, 140]:
    if t <= trajectory.total_time:
        state = trajectory.get_desired_state(t)
        pos = state['position']
        vel = state['velocity']
        print(f"{t:10.2f} | {pos[0]:12.1f} | {pos[2]:12.1f} | {vel[0]:12.2f} | {vel[2]:12.2f}")

print()
print("Z should go from 0 → 60,000m (UP!)")
print("If you see negative Z, the trajectory is BACKWARDS.")
