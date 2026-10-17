"""Steering/speed controllers (PLAN.md §5) and the map-free fallback (PLAN_REVIEW §4-1)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .track import TrackPath

G = 9.81


@dataclass
class VehicleParams:
    wheelbase: float = 0.145  # L
    track_width: float = 0.13  # t, rear wheel spacing
    width: float = 0.15
    length: float = 0.20
    max_steer: float = math.radians(30.0)  # needs >= 25.8 deg for R = 0.3 m


# --------------------------------------------------------------------------- Pure Pursuit

@dataclass
class PurePursuit:
    params: VehicleParams
    k_ld: float = 0.12  # L_d = k*v + ld0. Long L_d cuts the R=0.3 m corners into the inner wall
    ld0: float = 0.18
    ld_min: float = 0.18
    ld_max: float = 0.6
    _hint: int | None = None

    def lookahead(self, v: float) -> float:
        return float(np.clip(self.k_ld * v + self.ld0, self.ld_min, self.ld_max))

    def target(self, path: TrackPath, x: float, y: float, v: float) -> tuple[int, int]:
        """(nearest index, lookahead index). Keeps a hint so it never jumps across the track."""
        i = path.nearest_index(x, y, self._hint)
        self._hint = i
        return i, path.advance(i, self.lookahead(v))

    def steer(self, path: TrackPath, x: float, y: float, yaw: float, v: float) -> tuple[float, int]:
        i, j = self.target(path, x, y, v)
        tx, ty = path.xy[j]
        alpha = math.atan2(ty - y, tx - x) - yaw
        ld = max(math.hypot(tx - x, ty - y), 1e-3)
        delta = math.atan2(2.0 * self.params.wheelbase * math.sin(alpha), ld)
        return float(np.clip(delta, -self.params.max_steer, self.params.max_steer)), i

    def reset(self, hint: int | None = None) -> None:
        self._hint = hint


# --------------------------------------------------------------------------- vehicle helpers

def ediff(v: float, delta: float, p: VehicleParams, gain: float = 1.0) -> tuple[float, float]:
    """Rear wheel speeds (left, right). gain < 1 softens the split (PLAN_REVIEW: start 0.5-0.7)."""
    kappa = math.tan(delta) / p.wheelbase
    half = gain * kappa * p.track_width / 2.0
    return v * (1.0 - half), v * (1.0 + half)


def steer_speed_cap(delta: float, p: VehicleParams, a_lat: float = 0.7 * G) -> float:
    """v <= sqrt(a_lat * L / tan|delta|)  (same rule the ESP32 enforces)."""
    t = abs(math.tan(delta))
    if t < 1e-6:
        return float("inf")
    return math.sqrt(a_lat * p.wheelbase / t)


# --------------------------------------------------------------------------- Follow the Gap

@dataclass
class FollowTheGap:
    """Map-free fallback: steer toward the deepest reachable direction in the scan.

    Scan convention: angles in the vehicle frame (0 = forward, + = left), ranges in metres.
    Uses a disparity extender so the car does not cut corners into wall edges.
    """

    params: VehicleParams
    fov: float = math.radians(100)  # consider +-fov
    safety_margin: float = 0.05
    disparity_threshold: float = 0.15
    max_range: float = 4.0
    v_max: float = 1.5
    v_min: float = 0.4
    a_lat: float = 0.4 * G
    stop_distance: float = 0.10
    lookahead_min: float = 0.25
    lookahead_max: float = 0.8
    k_center: float = 0.6  # pulls toward the corridor middle when walls are close

    def command(self, angles: np.ndarray, ranges: np.ndarray) -> tuple[float, float]:
        r = np.clip(np.nan_to_num(ranges, nan=0.0, posinf=self.max_range), 0.0, self.max_range)
        order = np.argsort(angles)
        a, r = angles[order], r[order]

        ext = self._extend_disparities(a, r)
        mask = np.abs(a) <= self.fov
        cand = np.where(mask, ext, -1.0)
        # among near-max depths prefer the one closest to straight ahead
        best = np.max(cand)
        ok = cand >= best - 0.15
        idx = np.flatnonzero(ok)
        k = idx[np.argmin(np.abs(a[idx]))]
        heading = float(a[k])

        # centering: compare clearance to the left/right
        left = self._min_in(a, r, math.radians(60), math.radians(110))
        right = self._min_in(a, r, -math.radians(110), -math.radians(60))
        heading += self.k_center * float(np.clip(left - right, -0.3, 0.3))

        # Pure-Pursuit style arc toward the gap direction. Use a short fixed-ish lookahead,
        # not the gap depth: a 70 deg gap 1.5 m away would otherwise give ~10 deg of steer.
        front = self._min_in(a, r, -math.radians(12), math.radians(12))
        lookahead = float(np.clip(min(front, best), self.lookahead_min, self.lookahead_max))
        delta = math.atan2(2.0 * self.params.wheelbase * math.sin(heading), lookahead)
        delta = float(np.clip(delta, -self.params.max_steer, self.params.max_steer))

        if front < self.stop_distance:  # wall right at the bumper: emergency stop only
            return delta, 0.0
        v = min(self.v_max, steer_speed_cap(delta, self.params, self.a_lat))
        v = min(v, self.v_min + (front - self.stop_distance) * 1.2)
        # keep crawling while turning: stopping in a tight corner leaves the car stuck
        return delta, max(v, self.v_min)

    def _extend_disparities(self, a: np.ndarray, r: np.ndarray) -> np.ndarray:
        out = r.copy()
        half = self.params.width / 2.0 + self.safety_margin
        res = float(np.median(np.diff(a))) if len(a) > 1 else 0.01
        jumps = np.flatnonzero(np.abs(np.diff(r)) > self.disparity_threshold)
        for j in jumps:
            near = min(r[j], r[j + 1])
            if near <= 0:
                continue
            n = int(math.ceil(math.atan2(half, near) / max(res, 1e-4)))
            if r[j] < r[j + 1]:  # obstacle edge on the left-hand index side -> extend upward
                sl = slice(j + 1, min(len(r), j + 1 + n))
            else:
                sl = slice(max(0, j - n + 1), j + 1)
            out[sl] = np.minimum(out[sl], near)
        return out

    @staticmethod
    def _min_in(a: np.ndarray, r: np.ndarray, lo: float, hi: float) -> float:
        m = (a >= lo) & (a <= hi) & (r > 0)
        return float(np.min(r[m])) if np.any(m) else 0.0
