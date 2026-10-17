"""Offline speed profile along a path (PLAN.md §4)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .track import TrackPath

G = 9.81


@dataclass
class SpeedLimits:
    a_lat: float = 0.45 * G  # start conservative; PLAN_REVIEW §4: raise after tyre/floor test
    v_max: float = 3.0  # m/s
    a_acc: float = 2.5  # m/s^2
    a_dec: float = 3.0  # m/s^2
    v_min: float = 0.3
    # (s_start, s_end, v_cap) zones, e.g. speed bump
    zones: list[tuple[float, float, float]] = field(default_factory=list)


def bump_zone(scene: dict, before: float = 0.6, after: float = 0.3, v_cap: float = 1.0):
    """Slow-down zone around each official speed bump."""
    zones = []
    for b in scene["speed_bumps"]["bumps"]:
        zones.append((b["s_m"] - before, b["s_m"] + after, v_cap))
    return zones


def speed_profile(path: TrackPath, lim: SpeedLimits) -> np.ndarray:
    """Curvature cap, then forward (accel) and backward (decel) passes. Closed paths wrap."""
    kappa = np.abs(path.curvature)
    v = np.where(kappa > 1e-6, np.sqrt(lim.a_lat / np.maximum(kappa, 1e-6)), lim.v_max)
    v = np.minimum(v, lim.v_max)
    s_mod = path.s % path.length if path.closed else path.s
    for s0, s1, cap in lim.zones:
        if path.closed:
            s0, s1 = s0 % path.length, s1 % path.length
            mask = (s_mod >= s0) & (s_mod <= s1) if s0 <= s1 else (s_mod >= s0) | (s_mod <= s1)
        else:
            mask = (s_mod >= s0) & (s_mod <= s1)
        v[mask] = np.minimum(v[mask], cap)

    n = len(v)
    nxt = np.roll(path.xy, -1, axis=0)
    ds = np.linalg.norm(nxt - path.xy, axis=1)
    passes = 2 if path.closed else 1
    for _ in range(passes):
        rng = range(n) if path.closed else range(n - 1)
        for i in rng:
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2 * lim.a_acc * ds[i]))
        for i in reversed(rng):
            j = (i + 1) % n
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2 * lim.a_dec * ds[i]))
    return np.maximum(v, lim.v_min)


def lap_time(path: TrackPath, v: np.ndarray) -> float:
    nxt = np.roll(path.xy, -1, axis=0)
    ds = np.linalg.norm(nxt - path.xy, axis=1)
    v_avg = 0.5 * (v + np.roll(v, -1))
    if not path.closed:
        ds, v_avg = ds[:-1], v_avg[:-1]
    return float(np.sum(ds / v_avg))
