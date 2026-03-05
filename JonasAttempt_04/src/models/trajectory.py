"""
Reference trajectory generation for the ascent phase.

A quarter-ellipse from the launchpad to planned apogee provides
position & velocity references for the cascaded PID.
"""

import numpy as np
from src.config import X_DOWNRANGE, Z_APOGEE, T_COAST_END


class ReferenceTrajectory:
    """Time-parameterised quarter-ellipse in the X-Z plane (Y = 0).

    Uses Hermite interpolation for the time parameterisation so that
    **velocity is zero at both endpoints** (launch and apogee):

        τ = t / T_coast  ∈ [0, 1]
        f(τ) = 3τ² − 2τ³          (smooth start & stop)
        s(t) = (π/2) · f(τ)

    x(t) = X · (1 − cos s)
    z(t) = Z · sin s
    """

    def __init__(self,
                 x_down: float = X_DOWNRANGE,
                 z_apex: float = Z_APOGEE,
                 t_coast: float = T_COAST_END):
        self.x_down  = x_down
        self.z_apex  = z_apex
        self.t_coast = t_coast

    # ---- query at time t --------------------------------------------------

    def position(self, t: float) -> np.ndarray:
        s = self._s(t)
        return np.array([
            self.x_down * (1.0 - np.cos(s)),
            0.0,
            self.z_apex * np.sin(s),
        ])

    def velocity(self, t: float) -> np.ndarray:
        s = self._s(t)
        ds = self._ds(t)
        return np.array([
            self.x_down * np.sin(s) * ds,
            0.0,
            self.z_apex * np.cos(s) * ds,
        ])

    # ---- precompute for visualisation / export ----------------------------

    def sample(self, n: int = 500) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (x, y, z) arrays of the reference arc (for plotting)."""
        t = np.linspace(0, self.t_coast, n)
        tau = np.clip(t / self.t_coast, 0, 1)
        s = (np.pi / 2) * (3.0 * tau**2 - 2.0 * tau**3)
        x = self.x_down * (1.0 - np.cos(s))
        y = np.zeros_like(t)
        z = self.z_apex * np.sin(s)
        return x, y, z

    # ---- internals --------------------------------------------------------

    def _s(self, t: float) -> float:
        tau = np.clip(t / self.t_coast, 0.0, 1.0)
        f = 3.0 * tau**2 - 2.0 * tau**3          # Hermite: f(0)=0, f(1)=1, f'(0)=f'(1)=0
        return (np.pi / 2.0) * f

    def _ds(self, t: float) -> float:
        """ds/dt — needed for velocity = dx/ds · ds/dt."""
        if t < 0 or t > self.t_coast:
            return 0.0
        tau = t / self.t_coast
        # f'(τ) = 6τ(1−τ),  ds/dt = (π/2) · f'(τ) / T_coast
        f_prime = 6.0 * tau * (1.0 - tau)
        return (np.pi / 2.0) * f_prime / self.t_coast
