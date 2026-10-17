"""LaserScan -> vehicle-frame arrays (kept ROS-free so it can be unit tested)."""
from __future__ import annotations

import numpy as np


def scan_arrays(angle_min: float, angle_increment: float, ranges, yaw_offset: float = 0.0,
                time_increment: float = 0.0, scan_time: float = 0.0, flip: bool = False):
    """Returns (angles, ranges, beam_dt) in the vehicle frame (0 = forward, + = left).

    yaw_offset: lidar zero direction relative to the car's forward axis (mounting).
    flip: lidar mounted upside down.
    beam_dt: time of each beam relative to the end of the scan (<= 0), for de-skew.
    """
    r = np.asarray(ranges, dtype=float)
    n = len(r)
    a = angle_min + angle_increment * np.arange(n)
    if flip:
        a = -a
    a = (a + yaw_offset + np.pi) % (2 * np.pi) - np.pi
    r = np.where(np.isfinite(r), r, np.inf)
    if time_increment > 0:
        t = time_increment * np.arange(n)
        beam_dt = t - (scan_time if scan_time > 0 else t[-1])
    else:
        beam_dt = np.zeros(n)
    return a, r, beam_dt
