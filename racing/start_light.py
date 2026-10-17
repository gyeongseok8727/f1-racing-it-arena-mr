"""Start-light detection (official spec: 4 red then reds-off + green-on, MANUAL §6 / issue #3).

Event-based, never time-based: the 3rd and 4th red hold 1-3 s at random.
GO requires (1) at least `min_reds` reds seen lit earlier, then (2) green lit with
no red lit for `confirm_frames` consecutive frames. A green with no prior red
(e.g. the light's previous cycle, a reflection) is ignored -> no false start.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class StartState(Enum):
    WAITING = "waiting"  # nothing seen yet
    ARMED = "armed"  # reds observed
    GO = "go"


@dataclass
class LampReading:
    reds: int
    green: bool


@dataclass
class ColorThresholds:
    # HSV, OpenCV ranges (H 0-179). Lit LEDs are bright and saturated; tune on the real light.
    v_min: int = 170
    s_min: int = 90
    red_h: tuple[tuple[int, int], ...] = ((0, 10), (170, 179))
    green_h: tuple[int, int] = (45, 90)
    min_area_px: int = 12


def read_lamps(bgr_roi: np.ndarray, th: ColorThresholds = ColorThresholds()) -> LampReading:
    """Count lit red blobs and whether a lit green blob exists in the ROI around the light."""
    import cv2

    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    bright = (v >= th.v_min) & (s >= th.s_min)
    red = np.zeros_like(bright)
    for lo, hi in th.red_h:
        red |= (h >= lo) & (h <= hi)
    red &= bright
    green = bright & (h >= th.green_h[0]) & (h <= th.green_h[1])
    return LampReading(_count_blobs(red, th.min_area_px), _count_blobs(green, th.min_area_px) > 0)


def _count_blobs(mask: np.ndarray, min_area: int) -> int:
    import cv2

    n, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return int(np.sum(stats[1:, cv2.CC_STAT_AREA] >= min_area))


@dataclass
class StartDetector:
    min_reds: int = 1
    confirm_frames: int = 2
    state: StartState = StartState.WAITING
    max_reds_seen: int = 0
    _green_streak: int = field(default=0, repr=False)

    def update(self, reading: LampReading) -> StartState:
        if self.state is StartState.GO:
            return self.state
        if reading.reds > 0:
            self.max_reds_seen = max(self.max_reds_seen, reading.reds)
            if self.max_reds_seen >= self.min_reds:
                self.state = StartState.ARMED
            self._green_streak = 0
            return self.state
        if self.state is StartState.ARMED and reading.green:
            self._green_streak += 1
            if self._green_streak >= self.confirm_frames:
                self.state = StartState.GO
        else:
            self._green_streak = 0
        return self.state

    def reset(self) -> None:
        self.state = StartState.WAITING
        self.max_reds_seen = 0
        self._green_streak = 0
