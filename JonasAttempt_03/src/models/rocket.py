import numpy as np

from src import config
from src.utils import clamp


class Rocket:
    def __init__(self):
        self.mass = config.MASS
        self.length = config.LENGTH
        self.radius = config.RADIUS
        self.max_thrust = config.MAX_THRUST
        self.cg_from_gimbal = config.CG_FROM_GIMBAL
        self.inertia = config.INERTIA
        self.inertia_inv = np.linalg.inv(self.inertia)

    @property
    def gimbal_to_cg(self):
        return np.array([0.0, 0.0, self.cg_from_gimbal])

    @property
    def cp_offset(self):
        return np.array([0.0, 0.0, -config.CP_OFFSET_ASCENT])

    def cp_offset_for_phase(self, phase, grid_fins=0.0):
        if phase == "phase2":
            shift = config.CP_OFFSET_PHASE2 * clamp(grid_fins, 0.0, 1.0)
            return np.array([0.0, 0.0, shift])
        return self.cp_offset
