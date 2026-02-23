#!/usr/bin/env python3
import sys
sys.path.insert(0, '.')
from src.models.trajectory import VerticalLaunchPath

trajectory = VerticalLaunchPath(target_altitude=40_000.0)

# Check desired positions at different times
print("Desired trajectory points:")
for t in [0, 10, 20, 50, 90]:
    desired = trajectory.get_desired_state(t)
    pos_z = desired["position"][2]
    vel_z = desired["velocity"][2]
    print(f'Time {t:3d}s: Pos Z={pos_z:10.1f}m, Vel Z={vel_z:8.1f}m/s')
