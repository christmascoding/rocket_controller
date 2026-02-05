from dataclasses import dataclass
import numpy as np


@dataclass
class HelixTrajectory:
    radius_m: float
    height_m: float
    turns: float
    total_time: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Clamp time
        t = np.clip(t, 0.0, self.total_time)
        s = t / self.total_time
        theta = 2.0 * np.pi * self.turns * s

        x = self.radius_m * np.cos(theta)
        y = self.radius_m * np.sin(theta)
        z = self.height_m * s

        # Derivatives
        dtheta_dt = 2.0 * np.pi * self.turns / self.total_time
        dx = -self.radius_m * np.sin(theta) * dtheta_dt
        dy = self.radius_m * np.cos(theta) * dtheta_dt
        dz = self.height_m / self.total_time

        ddx = -self.radius_m * np.cos(theta) * dtheta_dt**2
        ddy = -self.radius_m * np.sin(theta) * dtheta_dt**2
        ddz = 0.0

        pos = np.array([x, y, z], dtype=float)
        vel = np.array([dx, dy, dz], dtype=float)
        acc = np.array([ddx, ddy, ddz], dtype=float)
        return pos, vel, acc


@dataclass
class UpwardCurveTrajectory:
    height_m: float
    curve_amp_m: float
    total_time: float

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = np.clip(t, 0.0, self.total_time)
        s = t / self.total_time
        # Slight lateral curve while moving upward
        x = self.curve_amp_m * np.sin(2.0 * np.pi * s)
        y = self.curve_amp_m * (1.0 - np.cos(2.0 * np.pi * s))
        z = self.height_m * s

        omega = 2.0 * np.pi / self.total_time
        dx = self.curve_amp_m * np.cos(2.0 * np.pi * s) * omega
        dy = self.curve_amp_m * np.sin(2.0 * np.pi * s) * omega
        dz = self.height_m / self.total_time

        ddx = -self.curve_amp_m * np.sin(2.0 * np.pi * s) * (2.0 * np.pi / self.total_time) ** 2
        ddy = self.curve_amp_m * np.cos(2.0 * np.pi * s) * (2.0 * np.pi / self.total_time) ** 2
        ddz = 0.0

        pos = np.array([x, y, z], dtype=float)
        vel = np.array([dx, dy, dz], dtype=float)
        acc = np.array([ddx, ddy, ddz], dtype=float)
        return pos, vel, acc

    def time_from_position(self, pos: np.ndarray) -> float:
        # Use height to estimate progress along the path
        s = np.clip(pos[2] / max(self.height_m, 1e-6), 0.0, 1.0)
        return s * self.total_time


@dataclass
class ParabolicCruiseTrajectory:
    peak_m: float
    ascent_length_m: float
    cruise_length_m: float
    descent_length_m: float
    ascent_time_s: float
    cruise_time_s: float
    descent_time_s: float

    @property
    def total_time(self) -> float:
        return self.ascent_time_s + self.cruise_time_s + self.descent_time_s

    def _smoothstep(self, s: float) -> float:
        return 3.0 * s**2 - 2.0 * s**3

    def _smoothstep_d(self, s: float) -> float:
        return 6.0 * s - 6.0 * s**2

    def _smoothstep_dd(self, s: float) -> float:
        return 6.0 - 12.0 * s

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = np.clip(t, 0.0, self.total_time)
        if t <= self.ascent_time_s:
            s = t / self.ascent_time_s
            x = self.ascent_length_m * s**2
            z = self.peak_m * (2.0 * s - s**2)
            dx = 2.0 * self.ascent_length_m * s / self.ascent_time_s
            dz = 2.0 * self.peak_m * (1.0 - s) / self.ascent_time_s
            ddx = 2.0 * self.ascent_length_m / (self.ascent_time_s**2)
            ddz = -2.0 * self.peak_m / (self.ascent_time_s**2)
        elif t <= self.ascent_time_s + self.cruise_time_s:
            tc = t - self.ascent_time_s
            s = tc / self.cruise_time_s
            x = self.ascent_length_m + self.cruise_length_m * s
            z = self.peak_m
            dx = self.cruise_length_m / self.cruise_time_s
            dz = 0.0
            ddx = 0.0
            ddz = 0.0
        else:
            td = t - self.ascent_time_s - self.cruise_time_s
            s = td / self.descent_time_s
            x = (
                self.ascent_length_m
                + self.cruise_length_m
                + self.descent_length_m * (2.0 * s - s**2)
            )
            z = self.peak_m * (1.0 - s**2)
            dx = 2.0 * self.descent_length_m * (1.0 - s) / self.descent_time_s
            dz = -2.0 * self.peak_m * s / self.descent_time_s
            ddx = -2.0 * self.descent_length_m / (self.descent_time_s**2)
            ddz = -2.0 * self.peak_m / (self.descent_time_s**2)

        # Ensure final point lands exactly at ground level
        if t >= self.total_time:
            x = self.ascent_length_m + self.cruise_length_m + self.descent_length_m
            z = 0.0
            dx = 0.0
            dz = 0.0
            ddx = 0.0
            ddz = 0.0

        pos = np.array([x, 0.0, z], dtype=float)
        vel = np.array([dx, 0.0, dz], dtype=float)
        acc = np.array([ddx, 0.0, ddz], dtype=float)
        return pos, vel, acc

    def time_from_position(self, pos: np.ndarray) -> float:
        x = pos[0]
        if x <= self.ascent_length_m:
            s = np.sqrt(np.clip(x / max(self.ascent_length_m, 1e-6), 0.0, 1.0))
            return s * self.ascent_time_s
        if x <= self.ascent_length_m + self.cruise_length_m:
            s = np.clip(
                (x - self.ascent_length_m) / max(self.cruise_length_m, 1e-6), 0.0, 1.0
            )
            return self.ascent_time_s + s * self.cruise_time_s
        u = np.clip(
            (x - self.ascent_length_m - self.cruise_length_m)
            / max(self.descent_length_m, 1e-6),
            0.0,
            1.0,
        )
        s = 1.0 - np.sqrt(np.clip(1.0 - u, 0.0, 1.0))
        return self.ascent_time_s + self.cruise_time_s + s * self.descent_time_s
