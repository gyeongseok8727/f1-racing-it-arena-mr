"""Map-based localization: wheel/gyro odometry, scan de-skew, likelihood-field particle filter.

Replaces F1TENTH particle_filter (range_libc build is fragile on Jetson/Humble) with a
numpy implementation testable in sim/sim2d.py. PLAN.md §3.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .track import OccupancyMap, wrap_angle


# --------------------------------------------------------------------------- odometry

@dataclass
class Odometry:
    """Integrates rear-wheel mean speed + IMU yaw rate (rear-axle reference)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    yaw_rate: float = 0.0

    def update(self, v_left: float, v_right: float, gyro_z: float, dt: float) -> tuple[float, float, float]:
        """Returns the motion delta (dx, dy, dyaw) in the *previous* body frame."""
        v = 0.5 * (v_left + v_right)
        dyaw = gyro_z * dt
        # midpoint integration
        h = dyaw / 2.0
        dx_b, dy_b = v * dt * math.cos(h), v * dt * math.sin(h)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * dx_b - s * dy_b
        self.y += s * dx_b + c * dy_b
        self.yaw = float(wrap_angle(self.yaw + dyaw))
        self.v, self.yaw_rate = v, gyro_z
        return dx_b, dy_b, dyaw


def relative_motion(prev: tuple[float, float, float], cur: tuple[float, float, float]) -> tuple[float, float, float]:
    """Delta between two odom poses expressed in the prev body frame."""
    dx, dy = cur[0] - prev[0], cur[1] - prev[1]
    c, s = math.cos(prev[2]), math.sin(prev[2])
    return c * dx + s * dy, -s * dx + c * dy, float(wrap_angle(cur[2] - prev[2]))


def deskew(angles: np.ndarray, ranges: np.ndarray, beam_dt: np.ndarray, v: float, yaw_rate: float):
    """Correct rotating-lidar motion distortion.

    beam_dt: time of each beam relative to the scan *end* (<= 0). Points are re-expressed in
    the sensor frame at scan end, assuming constant (v, yaw_rate) during the scan. At 3 m/s a
    10 Hz scan smears ~30 cm otherwise. Returns (angles, ranges) of corrected points.
    """
    px, py = ranges * np.cos(angles), ranges * np.sin(angles)
    # sensor pose at beam time relative to end pose (body moves +x by v*dt, rotates yaw_rate*dt)
    th = yaw_rate * beam_dt
    tx, ty = v * beam_dt * np.cos(th / 2), v * beam_dt * np.sin(th / 2)
    c, s = np.cos(th), np.sin(th)
    qx = c * px - s * py + tx
    qy = s * px + c * py + ty
    return np.arctan2(qy, qx), np.hypot(qx, qy)


# --------------------------------------------------------------------------- particle filter

@dataclass
class PFConfig:
    n_particles: int = 600
    n_beams: int = 72  # subsample of the 360-beam scan
    sigma_hit: float = 0.05  # m, likelihood-field std
    z_rand: float = 0.05  # random-measurement mixture weight
    max_range: float = 8.0
    lidar_x: float = 0.10  # lidar position forward of rear axle
    # motion noise (std per metre / per radian of motion + floor)
    a_trans: float = 0.10
    a_rot: float = 0.15
    a_rot_from_trans: float = 0.05
    floor_xy: float = 0.003
    floor_yaw: float = 0.003
    resample_neff: float = 0.5
    squash: float = 1.0 / 3.0  # tempers over-confident beam products (correlated beams)
    seed: int = 0


class ParticleFilter:
    def __init__(self, occ: OccupancyMap, cfg: PFConfig = PFConfig()):
        import cv2

        self.cfg = cfg
        self.map = occ
        self.rng = np.random.default_rng(cfg.seed)
        free = (~occ.occupied).astype(np.uint8)
        # distance (m) from each cell to the nearest wall/outside cell
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5).astype(np.float32) * occ.resolution
        self.particles = np.zeros((cfg.n_particles, 3))
        self.weights = np.full(cfg.n_particles, 1.0 / cfg.n_particles)
        self.last_score = 0.0

    # ---------------------------------------------------------------- init
    def initialize(self, x: float, y: float, yaw: float, std_xy: float = 0.05, std_yaw: float = 0.05) -> None:
        n = self.cfg.n_particles
        self.particles[:, 0] = x + self.rng.normal(0, std_xy, n)
        self.particles[:, 1] = y + self.rng.normal(0, std_xy, n)
        self.particles[:, 2] = yaw + self.rng.normal(0, std_yaw, n)
        self.weights[:] = 1.0 / n

    # ---------------------------------------------------------------- predict
    def predict(self, dx: float, dy: float, dyaw: float) -> None:
        c = self.cfg
        n = len(self.particles)
        trans = math.hypot(dx, dy)
        sx = c.a_trans * trans + c.floor_xy
        sy = c.a_trans * 0.5 * trans + c.floor_xy
        syaw = c.a_rot * abs(dyaw) + c.a_rot_from_trans * trans + c.floor_yaw
        ndx = dx + self.rng.normal(0, sx, n)
        ndy = dy + self.rng.normal(0, sy, n)
        ndyaw = dyaw + self.rng.normal(0, syaw, n)
        yaw = self.particles[:, 2]
        cs, sn = np.cos(yaw), np.sin(yaw)
        self.particles[:, 0] += cs * ndx - sn * ndy
        self.particles[:, 1] += sn * ndx + cs * ndy
        self.particles[:, 2] = wrap_angle(yaw + ndyaw)

    # ---------------------------------------------------------------- update
    def update(self, angles: np.ndarray, ranges: np.ndarray) -> None:
        c = self.cfg
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < c.max_range)
        idx = np.flatnonzero(valid)
        if len(idx) < 10:
            return
        if len(idx) > c.n_beams:
            idx = idx[np.linspace(0, len(idx) - 1, c.n_beams).astype(int)]
        a, r = angles[idx], ranges[idx]

        P = self.particles
        lx = P[:, 0] + c.lidar_x * np.cos(P[:, 2])
        ly = P[:, 1] + c.lidar_x * np.sin(P[:, 2])
        ang = P[:, 2:3] + a[None, :]
        ex = lx[:, None] + r[None, :] * np.cos(ang)
        ey = ly[:, None] + r[None, :] * np.sin(ang)
        d = self._dist_at(ex, ey)
        p_hit = np.exp(-0.5 * (d / c.sigma_hit) ** 2)
        p = (1 - c.z_rand) * p_hit + c.z_rand
        logw = c.squash * np.sum(np.log(p), axis=1)
        logw -= logw.max()
        w = self.weights * np.exp(logw)
        s = w.sum()
        if not np.isfinite(s) or s <= 0:
            return
        self.weights = w / s
        # fit score of the estimate: fraction of beams within 2 sigma of a wall
        self.last_score = float(np.sum(self.weights[:, None] * (d < 2 * c.sigma_hit)) / len(a))
        if self.neff() < c.resample_neff * len(P):
            self._resample()

    def _dist_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        m = self.map
        row, col = m.world_to_px(x, y)
        inside = (row >= 0) & (row < m.height) & (col >= 0) & (col < m.width)
        out = np.full(x.shape, 1.0, dtype=np.float32)
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    def neff(self) -> float:
        return float(1.0 / np.sum(self.weights**2))

    def _resample(self) -> None:
        n = len(self.particles)
        positions = (self.rng.random() + np.arange(n)) / n
        cum = np.cumsum(self.weights)
        cum[-1] = 1.0
        idx = np.searchsorted(cum, positions)
        self.particles = self.particles[idx]
        self.weights[:] = 1.0 / n

    # ---------------------------------------------------------------- outputs
    def estimate(self) -> tuple[float, float, float]:
        w = self.weights
        x = float(np.sum(w * self.particles[:, 0]))
        y = float(np.sum(w * self.particles[:, 1]))
        yaw = float(math.atan2(np.sum(w * np.sin(self.particles[:, 2])), np.sum(w * np.cos(self.particles[:, 2]))))
        return x, y, yaw

    def spread(self) -> tuple[float, float]:
        """(position std in m, yaw std in rad) — confidence used for fallback switching."""
        x, y, yaw = self.estimate()
        w = self.weights
        dxy = np.sum(w * ((self.particles[:, 0] - x) ** 2 + (self.particles[:, 1] - y) ** 2))
        dyaw = np.sum(w * wrap_angle(self.particles[:, 2] - yaw) ** 2)
        return float(math.sqrt(dxy)), float(math.sqrt(dyaw))

    def inject(self, x: float, y: float, yaw: float, fraction: float = 0.3, std_xy: float = 0.03, std_yaw: float = 0.03) -> None:
        """Absolute fix (e.g. ArUco PnP): replace a fraction of particles around the fix."""
        n = len(self.particles)
        k = int(n * fraction)
        idx = np.argsort(self.weights)[:k]
        self.particles[idx, 0] = x + self.rng.normal(0, std_xy, k)
        self.particles[idx, 1] = y + self.rng.normal(0, std_xy, k)
        self.particles[idx, 2] = yaw + self.rng.normal(0, std_yaw, k)
        self.weights[idx] = np.mean(self.weights)
        self.weights /= self.weights.sum()
