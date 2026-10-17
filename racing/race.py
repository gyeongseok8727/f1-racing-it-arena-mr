"""Race stack: start gating, map-mode Pure Pursuit with PF, Follow-the-Gap fallback,
opponent following, lap counting. Framework-free; ROS node and sim both drive it.

Call order (any rates):
    on_odom(v_left, v_right, gyro_z, dt)   ~50-100 Hz, from ESP32
    on_scan(angles, ranges)                ~10 Hz, lidar (vehicle frame, 0 = forward)
    on_start_light(go: bool)
    on_absolute_fix(x, y, yaw)             ArUco PnP, optional
    command() -> (steer_rad, speed_mps)    at control rate
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .control import FollowTheGap, PurePursuit, VehicleParams, steer_speed_cap
from .localization import Odometry, ParticleFilter, PFConfig, relative_motion
from .opponents import OpponentDetector, follow_speed_cap, project
from .planning import SpeedLimits, bump_zone, speed_profile
from .track import OccupancyMap, TrackPath, wrap_angle


class Phase(Enum):
    WAIT_START = "wait_start"
    RACING = "racing"
    FINISHED = "finished"


class Mode(Enum):
    MAP = "map"
    FALLBACK = "fallback"


@dataclass
class RaceConfig:
    laps: int = 5
    stop_after_finish: bool = True
    limits: SpeedLimits = field(default_factory=SpeedLimits)
    # fallback switching
    bad_spread_xy: float = 0.15
    bad_score: float = 0.45
    bad_scans_to_fallback: int = 3
    good_scans_to_recover: int = 10
    # opponents
    follow_enabled: bool = True
    lane_change_enabled: bool = False  # PLAN_REVIEW: follow first, lanes later
    # Road 0.45 m, car 0.15 m: a centred car leaves no room to pass on the road. +-0.19 m puts
    # our two outer wheels on grass, which is penalty-free (MANUAL §7: only >2 wheels count).
    lane_offsets: tuple[float, ...] = (0.0, 0.19, -0.19)
    lane_clearance: float = 0.17  # |lateral difference| that counts as "in our lane"
    look_ahead_obstacles: float = 2.0
    follow_standoff: float = 0.45  # rear axle -> their centre; ~0.18 m bumper gap
    min_lane_change_gap: float = 0.8
    pass_side_window: float = 0.45  # target lane must be clear this far behind our rear axle
    a_dec: float = 3.0
    # online wheel-odometry scale calibration
    calibrate_wheel_scale: bool = True
    calib_min_curvature: float = 0.8  # 1/m: only in corners, where the scan pins position
    calib_max_spread: float = 0.04
    calib_window: float = 0.5  # m of odometry per sample
    calib_alpha: float = 0.3


class RaceStack:
    def __init__(self, center: TrackPath, occ: OccupancyMap, scene: dict,
                 params: VehicleParams = VehicleParams(), cfg: RaceConfig | None = None,
                 pf_cfg: PFConfig = PFConfig()):
        self.p = params
        self.cfg = cfg or RaceConfig(limits=SpeedLimits(zones=bump_zone(scene)))
        if not self.cfg.limits.zones:
            self.cfg.limits.zones = bump_zone(scene)
        self.center = center
        self.lanes = [center if d == 0 else center.offset(d) for d in self.cfg.lane_offsets]
        self.v_ref = [speed_profile(l, self.cfg.limits) for l in self.lanes]
        self.lane = 0
        self.pp = PurePursuit(params)
        self.ftg = FollowTheGap(params)
        self.pf = ParticleFilter(occ, pf_cfg)
        self.det = OpponentDetector()
        self.odom = Odometry()
        self.wheel_scale = 1.0
        self._calib_pf = self._calib_odom = 0.0
        self._odom_at_scan = (0.0, 0.0, 0.0)
        self._pose_at_scan = (0.0, 0.0, 0.0)

        self.phase = Phase.WAIT_START
        self.mode = Mode.MAP
        self._bad, self._good = 0, 0
        self._scan = None
        self._obstacles = []
        # lap counting
        self._hint = None
        self._s_prev = None
        self.progress = 0.0
        self._to_line = 0.0
        self.laps_done = 0
        self.lap_times: list[float] = []
        self._t = 0.0
        self._lap_start = None

    # ---------------------------------------------------------------- setup
    def reset_pose(self, x: float, y: float, yaw: float, std_xy: float = 0.03) -> None:
        """x, y = rear axle. Grid slot poses are body centres: subtract wheelbase/2."""
        self.pf.initialize(x, y, yaw, std_xy, 0.03)
        self.odom = Odometry(x, y, yaw)
        self._odom_at_scan = (x, y, yaw)
        self._pose_at_scan = (x, y, yaw)
        self._hint = self.center.nearest_index(x, y)
        self._s_prev = self.center.s[self._hint]
        self._to_line = (self.center.length - self._s_prev) % self.center.length
        self.progress = 0.0
        self.pp.reset(self._hint)

    # ---------------------------------------------------------------- inputs
    def on_odom(self, v_left: float, v_right: float, gyro_z: float, dt: float) -> None:
        k = self.wheel_scale
        self.odom.update(v_left / k, v_right / k, gyro_z, dt)
        self._t += dt
        self._update_progress()

    def on_scan(self, angles: np.ndarray, ranges: np.ndarray) -> None:
        cur = (self.odom.x, self.odom.y, self.odom.yaw)
        dx, dy, dyaw = relative_motion(self._odom_at_scan, cur)
        self._odom_at_scan = cur
        self.pf.predict(dx, dy, dyaw)
        self.pf.update(angles, ranges)
        prev_pose = self._pose_at_scan
        self._pose_at_scan = self.pf.estimate()
        self._calibrate_wheel_scale(prev_pose, self._pose_at_scan, math.hypot(dx, dy))
        self._scan = (angles, ranges)
        self._update_mode()
        if self.cfg.follow_enabled and self.mode is Mode.MAP:
            obs = self.det.detect(self._pose_at_scan, angles, ranges, self.pf._dist_at, self.pf.cfg.lidar_x)
            self._obstacles = [project(self.center, o, self._hint) for o in obs]
        else:
            self._obstacles = []

    def _calibrate_wheel_scale(self, prev, cur, odom_dist: float) -> None:
        """Learn odometry scale (tyre compression, FG count error) where the PF is well
        constrained, so long straights — where lidar can't see along-track motion — drift less."""
        if not self.cfg.calibrate_wheel_scale:
            return
        if self._hint is None or abs(self.center.curvature[self._hint]) < self.cfg.calib_min_curvature:
            self._calib_pf = self._calib_odom = 0.0
            return
        spread, _ = self.pf.spread()
        if spread > self.cfg.calib_max_spread or self.pf.last_score < 0.7:
            return
        self._calib_pf += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        self._calib_odom += odom_dist
        if self._calib_odom >= self.cfg.calib_window:
            ratio = self._calib_pf / self._calib_odom  # true/odom with current scale applied
            k = self.wheel_scale / ratio
            k = float(np.clip(k, 0.85, 1.15))
            self.wheel_scale += self.cfg.calib_alpha * (k - self.wheel_scale)
            self._calib_pf = self._calib_odom = 0.0

    def on_start_light(self, go: bool) -> None:
        if go and self.phase is Phase.WAIT_START:
            self.phase = Phase.RACING

    def on_absolute_fix(self, x: float, y: float, yaw: float) -> None:
        self.pf.inject(x, y, yaw)

    # ---------------------------------------------------------------- state
    def pose(self) -> tuple[float, float, float]:
        """PF estimate at the last scan composed with odometry since then."""
        dx, dy, dyaw = relative_motion(self._odom_at_scan, (self.odom.x, self.odom.y, self.odom.yaw))
        x, y, yaw = self._pose_at_scan
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * dx - s * dy, y + s * dx + c * dy, float(wrap_angle(yaw + dyaw))

    def _update_mode(self) -> None:
        spread, _ = self.pf.spread()
        bad = spread > self.cfg.bad_spread_xy or self.pf.last_score < self.cfg.bad_score
        if bad:
            self._bad += 1
            self._good = 0
        else:
            self._good += 1
            self._bad = 0
        if self.mode is Mode.MAP and self._bad >= self.cfg.bad_scans_to_fallback:
            self.mode = Mode.FALLBACK
        elif self.mode is Mode.FALLBACK and self._good >= self.cfg.good_scans_to_recover:
            self.mode = Mode.MAP
            self.pp.reset(self._hint)

    def _update_progress(self) -> None:
        if self._s_prev is None:
            return
        x, y, _ = self.pose()
        self._hint = self.center.nearest_index(x, y, self._hint)
        s = self.center.s[self._hint]
        L = self.center.length
        self.progress += (s - self._s_prev + L / 2) % L - L / 2
        self._s_prev = s
        if self.phase is not Phase.RACING:
            return
        if self._lap_start is None and self.progress >= self._to_line:
            self._lap_start = self._t
        elif self._lap_start is not None and self.progress >= self._to_line + (self.laps_done + 1) * L:
            self.lap_times.append(self._t - self._lap_start)
            self._lap_start = self._t
            self.laps_done += 1
            if self.laps_done >= self.cfg.laps:
                self.phase = Phase.FINISHED

    # ---------------------------------------------------------------- command
    def command(self) -> tuple[float, float]:
        if self.phase is Phase.WAIT_START or self._scan is None:
            return 0.0, 0.0
        if self.phase is Phase.FINISHED and self.cfg.stop_after_finish:
            return 0.0, 0.0
        if self.mode is Mode.FALLBACK:
            return self.ftg.command(*self._scan)

        x, y, yaw = self.pose()
        v_now = max(self.odom.v, 0.5)
        self._choose_lane()
        path, vref = self.lanes[self.lane], self.v_ref[self.lane]
        delta, i = self.pp.steer(path, x, y, yaw, v_now)
        j = path.advance(i, 0.15 + 0.1 * v_now)
        v = min(vref[i], vref[j], steer_speed_cap(delta, self.p, self.cfg.limits.a_lat * 1.2))
        v = min(v, self._follow_cap())
        return delta, v

    def _ours(self) -> tuple[float, float]:
        x, y, yaw = self.pose()
        i = self._hint if self._hint is not None else self.center.nearest_index(x, y)
        h = self.center.heading[i]
        lat = -math.sin(h) * (x - self.center.xy[i, 0]) + math.cos(h) * (y - self.center.xy[i, 1])
        return float(self.center.s[i]), float(lat)

    def _blocking(self, lateral: float, behind: float = -0.05) -> list[tuple[float, object]]:
        """Obstacles within [behind, look_ahead] along the path (gap measured from our rear
        axle to their centre) whose lateral offset overlaps `lateral`."""
        s0, _ = self._ours()
        L = self.center.length
        out = []
        for o in self._obstacles:
            gap = (o.s - s0 + L / 2) % L - L / 2
            if behind < gap < self.cfg.look_ahead_obstacles and abs(o.lateral - lateral) < self.cfg.lane_clearance:
                out.append((gap, o))
        return out

    def _follow_cap(self) -> float:
        # check both the lane we aim for and where the car actually is (mid lane-change)
        _, lat_now = self._ours()
        block = self._blocking(self.cfg.lane_offsets[self.lane], 0.05) + self._blocking(lat_now, 0.05)
        if not block:
            return float("inf")
        gap = min(g for g, _ in block)
        return follow_speed_cap(gap, self.cfg.a_dec, standoff=self.cfg.follow_standoff)

    def _choose_lane(self) -> None:
        if not self.cfg.lane_change_enabled:
            self.lane = 0
            return
        offs = self.cfg.lane_offsets
        side = -self.cfg.pass_side_window  # also require the target lane clear beside us
        # return to the centre lane once it is clear (offset lanes put 2 wheels on grass)
        if self.lane != 0 and not self._blocking(offs[0], side):
            self._switch(0)
            return
        block = self._blocking(offs[self.lane], 0.05)
        if not block:
            return
        if min(g for g, _ in block) < self.cfg.min_lane_change_gap:
            return  # too close to move over safely: just follow
        for k, off in enumerate(offs):
            if k != self.lane and not self._blocking(off, side):
                self._switch(k)
                return

    def _switch(self, k: int) -> None:
        if k != self.lane:
            self.lane = k
            self.pp.reset(self._hint)
