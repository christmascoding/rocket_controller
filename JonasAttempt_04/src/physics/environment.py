"""
Atmosphere and gravity models.
"""
import numpy as np
from src.config import RHO_0, H_SCALE, G


def atmosphere_density(altitude: float) -> float:
    """Exponential atmosphere: ρ(h) = ρ₀ · exp(−h / H)."""
    if altitude < 0:
        altitude = 0.0
    return RHO_0 * np.exp(-altitude / H_SCALE)


def dynamic_pressure(altitude: float, speed: float) -> float:
    """q = ½ ρ v²."""
    return 0.5 * atmosphere_density(altitude) * speed ** 2


def gravity_force(mass: float) -> np.ndarray:
    """Constant-gravity force vector in world frame (pointing down)."""
    return np.array([0.0, 0.0, -mass * G])
