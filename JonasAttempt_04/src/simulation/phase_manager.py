"""
Phase manager — detects transition conditions and holds phase state.

Phases
------
'1a'  – Powered ascent (MECO at scheduled time)
'1b'  – Coast to apogee (engine off)
'1c'  – Boostback flip (MPC, partial thrust)
'2'   – Ballistic descent (ailerons only)
'3'   – Powered landing / hoverslam
'landed' – terminal
"""

import json, os
import numpy as np

from src.config import (
    MECO_TIME, FLIP_ANGLE_THRESHOLD, FLIP_TIMEOUT,
    SUICIDE_BURN_MARGIN, LANDING_ALT, LANDING_VEL,
    F_MAX, MASS, G,
)
from src.math_utils import quat_normalize, quat_to_dcm, safe_normalize


class PhaseManager:
    """Tracks the current phase and detects transitions."""

    def __init__(self):
        self.phase = '1a'
        self.phase_start_time = 0.0
        self.phase_log: list[tuple[float, str]] = [(0.0, '1a')]
        self.snapshots: dict[str, dict] = {}

    def update(self, t: float, state: np.ndarray) -> str:
        """Check for phase transition; return (possibly new) phase string."""
        prev = self.phase

        if self.phase == '1a':
            if t >= MECO_TIME:
                self._transition(t, '1b', state)

        elif self.phase == '1b':
            vz = state[5]
            alt = state[2]
            if vz <= 0.0 and alt > 1000.0:
                self._transition(t, '1c', state)

        elif self.phase == '1c':
            dt_phase = t - self.phase_start_time
            if self._is_retrograde(state) or dt_phase > FLIP_TIMEOUT:
                self._transition(t, '2', state)

        elif self.phase == '2':
            if self._should_burn(state):
                self._transition(t, '3', state)

        elif self.phase == '3':
            alt = state[2]
            vz  = state[5]
            if alt <= LANDING_ALT and vz >= -LANDING_VEL:
                self._transition(t, 'landed', state)

        return self.phase

    # ─────── helpers ──────────────────────────────────────────────────────

    def _transition(self, t, new_phase, state):
        self.phase = new_phase
        self.phase_start_time = t
        self.phase_log.append((t, new_phase))
        # save snapshot
        self.snapshots[new_phase] = {
            't': t,
            'state': state.tolist(),
        }
        print(f"  [Phase] {self.phase_log[-2][1]} → {new_phase}  at t = {t:.2f} s")

    def _is_retrograde(self, state) -> bool:
        """Body +z within threshold of −v."""
        vel  = state[3:6]
        quat = quat_normalize(state[6:10])
        R = quat_to_dcm(quat)
        body_z = R @ np.array([0.0, 0.0, 1.0])

        v_mag = np.linalg.norm(vel)
        if v_mag < 5.0:
            return True                # negligible velocity → call it done
        retro = -vel / v_mag
        cos_angle = np.clip(np.dot(body_z, retro), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return angle < FLIP_ANGLE_THRESHOLD

    def _should_burn(self, state) -> bool:
        """Suicide-burn trigger: altitude ≤ stopping distance × margin."""
        alt = state[2]
        vz  = state[5]
        speed_down = max(-vz, 0.0)
        if speed_down < 5.0 and alt > 500:
            return False
        a_max = F_MAX / MASS - G
        if a_max < 0.1:
            return False
        d_stop = speed_down**2 / (2.0 * a_max)
        return alt <= d_stop * SUICIDE_BURN_MARGIN

    # ─────── snapshot I/O ─────────────────────────────────────────────────

    def save_snapshots(self, path: str):
        with open(path, 'w') as f:
            json.dump(self.snapshots, f, indent=2)

    @staticmethod
    def load_snapshot(path: str, phase: str) -> dict:
        with open(path, 'r') as f:
            data = json.load(f)
        return data[phase]
