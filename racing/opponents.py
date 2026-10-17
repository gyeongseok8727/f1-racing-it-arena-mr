"""Opponent detection from the scan: points that are not explained by the map (PLAN.md §4)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .track import TrackPath


@dataclass
class Obstacle:
    x: float
    y: float
    n_points: int
    s: float = 0.0  # arc length on the reference path
    lateral: float = 0.0  # signed offset from the reference path (left +)


@dataclass
class OpponentDetector:
    min_wall_distance: float = 0.10  # endpoints closer than this to a wall belong to the map
    cluster_gap: float = 0.10
    min_points: int = 3
    max_range: float = 3.0

    def detect(self, pose: tuple[float, float, float], angles: np.ndarray, ranges: np.ndarray,
               dist_field_at, lidar_x: float = 0.10) -> list[Obstacle]:
        """dist_field_at(x, y) -> distance to nearest mapped wall (ParticleFilter._dist_at)."""
        x, y, yaw = pose
        ok = np.isfinite(ranges) & (ranges > 0.05) & (ranges < self.max_range)
        if not np.any(ok):
            return []
        a, r = angles[ok], ranges[ok]
        order = np.argsort(a)
        a, r = a[order], r[order]
        lx, ly = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
        ex = lx + r * np.cos(a + yaw)
        ey = ly + r * np.sin(a + yaw)
        unexplained = dist_field_at(ex, ey) > self.min_wall_distance
        pts = np.column_stack([ex, ey])[unexplained]
        if len(pts) == 0:
            return []
        clusters, cur = [], [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - cur[-1]) <= self.cluster_gap:
                cur.append(p)
            else:
                clusters.append(cur)
                cur = [p]
        clusters.append(cur)
        out = []
        for c in clusters:
            if len(c) >= self.min_points:
                c = np.asarray(c)
                out.append(Obstacle(float(c[:, 0].mean()), float(c[:, 1].mean()), len(c)))
        return out


def project(path: TrackPath, obs: Obstacle, hint: int | None = None) -> Obstacle:
    i = path.nearest_index(obs.x, obs.y, hint, window=300) if hint is not None else path.nearest_index(obs.x, obs.y)
    h = path.heading[i]
    dx, dy = obs.x - path.xy[i, 0], obs.y - path.xy[i, 1]
    obs.s = float(path.s[i])
    obs.lateral = float(-math.sin(h) * dx + math.cos(h) * dy)
    return obs


def follow_speed_cap(gap: float, a_dec: float = 3.0, standoff: float = 0.30, react: float = 0.15,
                     v_obs: float = 0.0) -> float:
    """Max speed so we can stop `standoff` behind an obstacle `gap` metres ahead (centre to
    centre along the path), assuming a reaction time and that the obstacle may stop."""
    room = gap - standoff
    if room <= 0:
        return 0.0
    # v*react + v^2/(2a) = room + v_obs^2/(2a)
    b = react
    c = -(room + v_obs**2 / (2 * a_dec))
    return float((-b + math.sqrt(b * b - 4 * (1 / (2 * a_dec)) * c)) / (2 * (1 / (2 * a_dec))))
