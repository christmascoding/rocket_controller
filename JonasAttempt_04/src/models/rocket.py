"""
TTHopper rocket model — collects all vehicle parameters in one object
so they can be passed around cleanly.
"""

import numpy as np
from src.config import (
    MASS, LENGTH, CG_FROM_BOTTOM, RADIUS,
    INERTIA, INERTIA_INV, F_MAX, GIMBAL_MAX, GIMBAL_TO_CG,
    CP_OFFSET_ASCENT, CP_OFFSET_DESCENT
)


class Rocket:
    """Immutable data-object that describes the TTHopper vehicle."""

    def __init__(self):
        self.mass          = MASS
        self.length        = LENGTH
        self.cg_height     = CG_FROM_BOTTOM
        self.radius        = RADIUS
        self.inertia       = INERTIA
        self.inertia_inv   = INERTIA_INV
        self.f_max         = F_MAX
        self.gimbal_max    = GIMBAL_MAX
        self.gimbal_to_cg  = GIMBAL_TO_CG

    # convenience -----------------------------------------------------------
    @property
    def thrust_to_weight(self) -> float:
        from src.config import G
        return self.f_max / (self.mass * G)

    def cp_offset(self, phase: str) -> np.ndarray:
        if phase in ('2', '3', 'landed'):
            return np.array([0.0, 0.0, CP_OFFSET_DESCENT])
        return np.array([0.0, 0.0, CP_OFFSET_ASCENT])

    def __repr__(self):
        return (f"Rocket(mass={self.mass} kg, L={self.length} m, "
                f"Fmax={self.f_max/1e3:.0f} kN, T/W={self.thrust_to_weight:.2f})")
