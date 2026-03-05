"""
Data logger — stores time-series from each simulation step and
exports the full history to JSON for offline visualisation.
"""

import json
import numpy as np
from src.math_utils import quat_to_euler


class DataLogger:
    """Append-only time-series store with JSON export."""

    def __init__(self):
        # core series
        self.time:      list[float] = []
        self.phase:     list[str]   = []
        # state
        self.x:  list[float] = [];  self.y:  list[float] = [];  self.z:  list[float] = []
        self.vx: list[float] = [];  self.vy: list[float] = [];  self.vz: list[float] = []
        self.phi: list[float] = []; self.theta: list[float] = []; self.psi: list[float] = []
        self.p:  list[float] = [];  self.q: list[float] = [];  self.r:  list[float] = []
        self.qw: list[float] = [];  self.qx: list[float] = []
        self.qy: list[float] = [];  self.qz: list[float] = []
        # control
        self.throttle: list[float] = []
        self.gimbal_y: list[float] = [];  self.gimbal_z: list[float] = []
        self.aileron_x: list[float] = []; self.aileron_y: list[float] = []
        self.aileron_z: list[float] = []
        # forces / moments  (world / body)
        self.Ftx: list[float] = []; self.Fty: list[float] = []; self.Ftz: list[float] = []
        self.Fax: list[float] = []; self.Fay: list[float] = []; self.Faz: list[float] = []
        self.Mtx: list[float] = []; self.Mty: list[float] = []; self.Mtz: list[float] = []
        self.Max: list[float] = []; self.May: list[float] = []; self.Maz: list[float] = []
        # debug extras (variable-length per controller, stored as dicts)
        self.debug: list[dict] = []

    # ──────────────────────────────────────────────────────────────────────

    def log(self, t: float, state: np.ndarray, control: np.ndarray,
            phase: str, forces: dict | None = None,
            debug: dict | None = None):
        self.time.append(t)
        self.phase.append(phase)

        # position / velocity
        self.x.append(float(state[0]));  self.y.append(float(state[1]))
        self.z.append(float(state[2]))
        self.vx.append(float(state[3])); self.vy.append(float(state[4]))
        self.vz.append(float(state[5]))

        # quaternion
        qw, qx, qy, qz = state[6:10]
        self.qw.append(float(qw)); self.qx.append(float(qx))
        self.qy.append(float(qy)); self.qz.append(float(qz))

        # euler (display only)
        euler = quat_to_euler(state[6:10])
        self.phi.append(float(euler[0]))
        self.theta.append(float(euler[1]))
        self.psi.append(float(euler[2]))

        # angular rates
        self.p.append(float(state[10])); self.q.append(float(state[11]))
        self.r.append(float(state[12]))

        # control
        self.throttle.append(float(control[0]))
        self.gimbal_y.append(float(control[1])); self.gimbal_z.append(float(control[2]))
        self.aileron_x.append(float(control[3])); self.aileron_y.append(float(control[4]))
        self.aileron_z.append(float(control[5]))

        # forces (optional)
        if forces:
            ft = forces.get('F_thrust_world', np.zeros(3))
            fa = forces.get('F_aero_world', np.zeros(3))
            mt = forces.get('M_thrust_body', np.zeros(3))
            ma = forces.get('M_aero_body', np.zeros(3))
            self.Ftx.append(float(ft[0])); self.Fty.append(float(ft[1])); self.Ftz.append(float(ft[2]))
            self.Fax.append(float(fa[0])); self.Fay.append(float(fa[1])); self.Faz.append(float(fa[2]))
            self.Mtx.append(float(mt[0])); self.Mty.append(float(mt[1])); self.Mtz.append(float(mt[2]))
            self.Max.append(float(ma[0])); self.May.append(float(ma[1])); self.Maz.append(float(ma[2]))
        else:
            for lst in (self.Ftx, self.Fty, self.Ftz,
                        self.Fax, self.Fay, self.Faz,
                        self.Mtx, self.Mty, self.Mtz,
                        self.Max, self.May, self.Maz):
                lst.append(0.0)

        # debug (serialise numpy arrays)
        self.debug.append(_sanitise(debug) if debug else {})

    # ──────────────────────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Return entire history as a plain dict (JSON-serialisable)."""
        return {
            'time': self.time, 'phase': self.phase,
            'states': {
                'x': self.x, 'y': self.y, 'z': self.z,
                'vx': self.vx, 'vy': self.vy, 'vz': self.vz,
                'phi': self.phi, 'theta': self.theta, 'psi': self.psi,
                'p': self.p, 'q': self.q, 'r': self.r,
                'qw': self.qw, 'qx': self.qx, 'qy': self.qy, 'qz': self.qz,
            },
            'controls': {
                'throttle': self.throttle,
                'gimbal_y': self.gimbal_y, 'gimbal_z': self.gimbal_z,
                'aileron_x': self.aileron_x, 'aileron_y': self.aileron_y,
                'aileron_z': self.aileron_z,
            },
            'forces': {
                'Ftx': self.Ftx, 'Fty': self.Fty, 'Ftz': self.Ftz,
                'Fax': self.Fax, 'Fay': self.Fay, 'Faz': self.Faz,
                'Mtx': self.Mtx, 'Mty': self.Mty, 'Mtz': self.Mtz,
                'Max': self.Max, 'May': self.May, 'Maz': self.Maz,
            },
            'debug': self.debug,
        }

    def save_json(self, path: str):
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f)
        print(f"  [Logger] Saved {len(self.time)} samples → {path}")


# ─── helper ──────────────────────────────────────────────────────────────

def _sanitise(d: dict) -> dict:
    """Convert numpy types to plain Python for JSON."""
    out = {}
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, (np.floating, np.integer)):
            out[k] = float(v)
        else:
            out[k] = v
    return out
