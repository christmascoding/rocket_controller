"""
Abstract base for all flight-phase controllers.
"""
from abc import ABC, abstractmethod
import numpy as np

# wir haben jetzet einfach für die controller eine base klasse und das passt dann müssen wir nicht schon wieder wie beim letzten mal alles von vorne aufbauen
# wow you discovered object oriented programming!!! <3
class BaseController(ABC):
    """Every phase controller exposes a single :meth:`compute` method.

    Parameters
    ----------
    t     : simulation time  [s]
    state : 13-element state vector
    info  : dict with additional context the simulator provides, such as
            ``ref_pos``, ``ref_vel``, ``phase_time``, etc.

    Returns
    -------
    control : 6-element vector
        [throttle, gimbal_y, gimbal_z, aileron_τx, aileron_τy, aileron_τz]
    debug   : dict  — arbitrary debug data passed to the logger
    """

    @abstractmethod
    def compute(self, t: float, state: np.ndarray,
                info: dict) -> tuple[np.ndarray, dict]:
        ...
