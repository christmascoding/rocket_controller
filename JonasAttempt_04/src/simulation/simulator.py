"""
Main simulation loop — integrates the 6-DOF plant with RK4,
dispatches to the correct phase controller, and logs everything.
"""

import time as _time
import numpy as np

from src.config import DT, T_MAX, LOG_DECIMATION
from src.math_utils import quat_normalize
from src.physics.dynamics import state_derivative, compute_forces_moments
from src.models.trajectory import ReferenceTrajectory
from src.simulation.phase_manager import PhaseManager
from src.simulation.data_logger import DataLogger

from src.controllers.ascent_pid import AscentPID
from src.controllers.flip_mpc import FlipMPC
from src.controllers.ballistic_smc import BallisticSMC
from src.controllers.landing_lqr import LandingLQR


class Simulator:
    """RK4-integration loop with phase management."""

    def __init__(self, dt: float = DT, t_max: float = T_MAX):
        self.dt    = dt
        self.t_max = t_max

        # phase manager
        self.pm = PhaseManager()

        # controllers (one per cluster of phases)
        self.ctrl_ascent    = AscentPID()
        self.ctrl_flip      = FlipMPC()
        self.ctrl_ballistic = BallisticSMC()
        self.ctrl_landing   = LandingLQR()

        # reference trajectory (for phase 1a tracking)
        self.ref_traj = ReferenceTrajectory()

        # data logging
        self.logger = DataLogger()

    # ──────────────────────── public API ──────────────────────────────────

    def run_from_phase(self, phase: str, t0: float,
                       state0: np.ndarray) -> DataLogger:
        """Resume simulation from *phase* at time *t0* with *state0*.

        Useful for fast iteration: run once from scratch, save snapshot,
        then reload and re-simulate only the later phases.
        """
        self.pm.phase = phase
        self.pm.phase_start_time = t0
        self.pm.phase_log = [(t0, phase)]
        return self.run(state0=state0, t0=t0)

    def run(self, state0: np.ndarray | None = None,
            t0: float = 0.0) -> DataLogger:
        """Run the full simulation. Returns the filled DataLogger."""
        if state0 is None:
            state0 = self._default_initial_state()
        state = state0.copy()

        t = t0
        step = 0
        wall0 = _time.perf_counter()

        print(f"  [Sim] Starting — dt={self.dt}  t_max={self.t_max}")

        while t < self.t_max and self.pm.phase != 'landed':
            # --- phase check ---
            self.pm.update(t, state)
            phase = self.pm.phase
            if phase == 'landed':
                break

            # --- controller ---
            info = self._build_info(t, state, phase)
            control, debug = self._dispatch(t, state, phase, info)

            # --- log (decimated) ---
            if step % LOG_DECIMATION == 0:
                forces = compute_forces_moments(state, control, phase)
                self.logger.log(t, state, control, phase, forces, debug)

            # --- RK4 integration ---
            state = self._rk4_step(state, control, phase)

            t += self.dt
            step += 1

            # safety: ground clamp
            if state[2] < 0.0:
                state[2] = 0.0
                if state[5] < 0.0:
                    state[5] = 0.0
                break

        # final log entry
        control_zero = np.zeros(6)
        forces_final = compute_forces_moments(state, control_zero, self.pm.phase)
        self.logger.log(t, state, control_zero, self.pm.phase, forces_final, {})

        wall = _time.perf_counter() - wall0
        print(f"  [Sim] Done — t_final={t:.2f} s  steps={step}  "
              f"wall={wall:.1f} s  phase={self.pm.phase}")
        return self.logger

    # ──────────────────────── internals ───────────────────────────────────

    @staticmethod
    def _default_initial_state() -> np.ndarray:
        """Rocket on the pad, vertical, at rest."""
        return np.array([
            0.0, 0.0, 0.0,          # position
            0.0, 0.0, 0.0,          # velocity
            1.0, 0.0, 0.0, 0.0,     # quaternion (identity → z_B = world Z)
            0.0, 0.0, 0.0,          # angular velocity
        ])

    def _build_info(self, t, state, phase) -> dict:
        info = {'phase': phase, 'phase_time': t - self.pm.phase_start_time}
        if phase == '1a':
            info['ref_pos'] = self.ref_traj.position(t)
            info['ref_vel'] = self.ref_traj.velocity(t)
        return info

    def _dispatch(self, t, state, phase, info):
        if phase in ('1a', '1b'):
            return self.ctrl_ascent.compute(t, state, info)
        elif phase == '1c':
            return self.ctrl_flip.compute(t, state, info)
        elif phase == '2':
            return self.ctrl_ballistic.compute(t, state, info)
        elif phase == '3':
            return self.ctrl_landing.compute(t, state, info)
        else:
            return np.zeros(6), {}

    def _rk4_step(self, state, control, phase):
        """Classical 4th-order Runge-Kutta with quaternion renormalisation."""
        dt = self.dt
        k1 = state_derivative(state,               control, phase)
        k2 = state_derivative(state + 0.5 * dt * k1, control, phase)
        k3 = state_derivative(state + 0.5 * dt * k2, control, phase)
        k4 = state_derivative(state + dt * k3,       control, phase)
        s_new = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        # renormalise quaternion
        s_new[6:10] = quat_normalize(s_new[6:10])
        return s_new
