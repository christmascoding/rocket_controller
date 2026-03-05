"""
Snapshot save / load for the tuning framework.

At each phase transition the simulator writes a JSON snapshot containing
the full 13-DOF state, the time stamp, environment conditions, and
the phase-specific target.  A headless tuning script can then reload
this snapshot to run a single phase hundreds of times.
"""

import json
import numpy as np


def save_snapshot(path: str, phase: str, t: float,
                  state: np.ndarray, extra: dict | None = None):
    """Persist a phase-entry snapshot to *path*."""
    payload = {
        'phase': phase,
        't': t,
        'state': state.tolist(),
    }
    if extra:
        for k, v in extra.items():
            if isinstance(v, np.ndarray):
                payload[k] = v.tolist()
            else:
                payload[k] = v

    with open(path, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f"  [Snapshot] Saved phase={phase} at t={t:.2f} → {path}")


def load_snapshot(path: str) -> dict:
    """Load snapshot; 'state' is returned as an np.ndarray."""
    with open(path, 'r') as f:
        data = json.load(f)
    data['state'] = np.array(data['state'])
    return data
