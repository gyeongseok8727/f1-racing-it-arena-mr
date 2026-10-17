"""Lightweight 2D sim on the official occupancy map (no ROS, no Gazebo).

Kinematic bicycle (rear-axle reference) + 2D lidar raycast + noisy wheel/gyro sensors.
Checks: wall/car contact, grass entries (>2 wheels on grass = 1 penalty, MANUAL §7), lap times.
Purpose is logic validation, not physics fidelity (PLAN_REVIEW §3).

    python -m sim.sim2d --mode stack --laps 5 --plot out/stack.png        # full race stack (PF)
    python -m sim.sim2d --mode stack --opponents 1 --plot out/follow.png  # + slower car ahead
    python -m sim.sim2d --mode pp    # ground-truth pose Pure Pursuit (upper bound)
    python -m sim.sim2d --mode ftg   # map-free fallback only
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from racing.control import FollowTheGap, PurePursuit, VehicleParams, steer_speed_cap
from racing.planning import SpeedLimits, bump_zone, lap_time, speed_profile
from racing.race import Mode, RaceConfig, RaceStack
from racing.track import (
    OFFICIAL_DIR,
    PX_GRASS,
    OccupancyMap,
    grid_slot_pose,
    load_centerline,
    load_scene,
)


@dataclass
class SimConfig:
    dt: float = 0.01
    control_dt: float = 0.02  # 50 Hz command to ESP32
    scan_dt: float = 0.1  # RPLIDAR C1 ~10 Hz
    n_beams: int = 360
    max_range: float = 12.0
    range_noise: float = 0.01
    lidar_x: float = 0.10  # forward of rear axle
    steer_rate: float = math.radians(360)  # rad/s servo slew
    a_max: float = 3.0
    d_max: float = 4.0
    time_limit: float = 200.0
    # sensor errors seen by the stack
    wheel_noise: float = 0.02  # m/s
    wheel_scale: float = 1.03  # TPU tyre compression: odometry over-reads distance
    gyro_bias: float = 0.01  # rad/s
    gyro_noise: float = 0.02
    seed: int = 0


@dataclass
class Opponent:
    s: float  # start arc length on the centreline
    speed: float
    lateral: float = 0.0
    hint: int | None = None


@dataclass
class SimResult:
    laps: list[float] = field(default_factory=list)
    crashed: bool = False
    crash_time: float | None = None
    crash_with: str = ""
    grass_penalties: int = 0
    max_speed: float = 0.0
    min_opponent_gap: float = float("inf")
    max_loc_error: float = 0.0
    fallback_time: float = 0.0
    trace: list[tuple[float, float, float]] = field(default_factory=list)

    def summary(self) -> str:
        laps = ", ".join(f"{t:.2f}" for t in self.laps) or "-"
        state = f"CRASH({self.crash_with}) at {self.crash_time:.2f}s" if self.crashed else "ok"
        extra = ""
        if self.max_loc_error:
            extra += f" | max loc err {self.max_loc_error * 100:.1f} cm | fallback {self.fallback_time:.1f}s"
        if math.isfinite(self.min_opponent_gap):
            extra += f" | min gap to opp {self.min_opponent_gap:.2f} m"
        return (f"laps [{laps}] s | {state} | grass {self.grass_penalties} "
                f"| vmax {self.max_speed:.2f} m/s{extra}")


class Sim:
    def __init__(self, cfg: SimConfig | None = None, params: VehicleParams | None = None):
        self.cfg, self.p = cfg or SimConfig(), params or VehicleParams()
        self.map = OccupancyMap.load(OFFICIAL_DIR / "map.yaml")
        self.grass = OccupancyMap.load(OFFICIAL_DIR / "map_with_grass.yaml")
        self.center = load_centerline()
        self.scene = load_scene()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.beam_angles = np.linspace(-math.pi, math.pi, self.cfg.n_beams, endpoint=False)
        self.opponents: list[Opponent] = []

    # ---------------------------------------------------------------- geometry
    def body_corners(self, x, y, yaw, rear_axle=True):
        L, W, l = self.p.wheelbase, self.p.width, self.p.length
        cx = L / 2 if rear_axle else 0.0
        pts = np.array([(cx - l / 2, -W / 2), (cx + l / 2, -W / 2), (cx + l / 2, W / 2), (cx - l / 2, W / 2)])
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])

    def footprint(self, x, y, yaw, n=4):
        """Sample points on the body outline (body centred between the axles)."""
        cs = self.body_corners(x, y, yaw)
        pts = [cs[k] + (cs[(k + 1) % 4] - cs[k]) * u for k in range(4) for u in np.linspace(0, 1, n, endpoint=False)]
        pts = np.asarray(pts)
        return pts[:, 0], pts[:, 1]

    def wheels(self, x, y, yaw):
        L, t = self.p.wheelbase, self.p.track_width
        pts = np.array([(0, t / 2), (0, -t / 2), (L, t / 2), (L, -t / 2)])
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]

    # ---------------------------------------------------------------- opponents
    def add_opponent(self, s: float, speed: float, lateral: float = 0.0) -> None:
        self.opponents.append(Opponent(s % self.center.length, speed, lateral))

    def _opp_pose(self, o: Opponent):
        i = self.center.index_at_s(o.s)
        h = self.center.heading[i]
        x = self.center.xy[i, 0] - math.sin(h) * o.lateral
        y = self.center.xy[i, 1] + math.cos(h) * o.lateral
        return x, y, h

    def _dynamic_mask(self):
        if not self.opponents:
            return None
        import cv2

        mask = self.map.occupied.astype(np.uint8)
        for o in self.opponents:
            cs = self.body_corners(*self._opp_pose(o), rear_axle=False)
            row, col = self.map.world_to_px(cs[:, 0], cs[:, 1])
            cv2.fillPoly(mask, [np.column_stack([col, row]).astype(np.int32)], 1)
        return mask.astype(bool)

    def scan(self, x, y, yaw):
        lx, ly = x + self.cfg.lidar_x * math.cos(yaw), y + self.cfg.lidar_x * math.sin(yaw)
        r = self.map.raycast(lx, ly, self.beam_angles + yaw, self.cfg.max_range, self._dynamic_mask())
        return r + self.rng.normal(0, self.cfg.range_noise, r.shape)

    # ---------------------------------------------------------------- run
    def start_pose(self, slot: int):
        x, y, yaw = grid_slot_pose(slot, self.scene)  # body centre -> rear axle
        return x - math.cos(yaw) * self.p.wheelbase / 2, y - math.sin(yaw) * self.p.wheelbase / 2, yaw

    def run(self, controller, laps: int = 5, slot: int = 0) -> SimResult:
        cfg = self.cfg
        x, y, yaw = self.start_pose(slot)
        v, delta = 0.0, 0.0
        cmd_v, cmd_delta = 0.0, 0.0
        res = SimResult()
        Ltrack = self.center.length
        hint = self.center.nearest_index(x, y)
        s_prev = self.center.s[hint]
        progress = 0.0
        first_line = (Ltrack - s_prev) % Ltrack
        lap_start, laps_done = None, 0
        on_grass = False
        t = 0.0
        next_ctrl, next_scan = 0.0, 0.0
        controller.reset(x, y, yaw)

        while t < cfg.time_limit and laps_done < laps:
            # sensors
            yaw_rate = v * math.tan(delta) / self.p.wheelbase
            kappa_v = yaw_rate * self.p.track_width / 2
            n = self.rng.normal(0, cfg.wheel_noise, 2)
            wl = (v - kappa_v) * cfg.wheel_scale + n[0]
            wr = (v + kappa_v) * cfg.wheel_scale + n[1]
            gz = yaw_rate + cfg.gyro_bias + self.rng.normal(0, cfg.gyro_noise)
            controller.on_odom(wl, wr, gz, cfg.dt)
            if t >= next_scan:
                controller.on_scan(self.beam_angles, self.scan(x, y, yaw))
                next_scan += cfg.scan_dt
            if t >= next_ctrl:
                cmd_delta, cmd_v = controller.command((x, y, yaw, v))
                next_ctrl += cfg.control_dt
                est = controller.estimate()
                if est is not None and controller.racing():
                    res.max_loc_error = max(res.max_loc_error, math.hypot(est[0] - x, est[1] - y))
                if controller.in_fallback():
                    res.fallback_time += cfg.control_dt

            # actuators + kinematics
            dd = np.clip(cmd_delta - delta, -cfg.steer_rate * cfg.dt, cfg.steer_rate * cfg.dt)
            delta = float(np.clip(delta + dd, -self.p.max_steer, self.p.max_steer))
            dv = np.clip(cmd_v - v, -cfg.d_max * cfg.dt, cfg.a_max * cfg.dt)
            v = max(0.0, v + float(dv))
            x += v * math.cos(yaw) * cfg.dt
            y += v * math.sin(yaw) * cfg.dt
            yaw += v * math.tan(delta) / self.p.wheelbase * cfg.dt
            for o in self.opponents:
                o.s = (o.s + o.speed * cfg.dt) % Ltrack
            t += cfg.dt
            res.max_speed = max(res.max_speed, v)

            # contacts
            fx, fy = self.footprint(x, y, yaw)
            if np.any(self.map.is_occupied(fx, fy)):
                res.crashed, res.crash_time, res.crash_with = True, t, "wall"
                break
            if self._hits_opponent(x, y, yaw, res):
                res.crashed, res.crash_time, res.crash_with = True, t, "car"
                break
            wx, wy = self.wheels(x, y, yaw)
            grass_now = int(np.sum(self.grass.value_at(wx, wy) == PX_GRASS)) > 2
            if grass_now and not on_grass:
                res.grass_penalties += 1
            on_grass = grass_now

            # laps (truth)
            hint = self.center.nearest_index(x, y, hint)
            s_now = self.center.s[hint]
            progress += (s_now - s_prev + Ltrack / 2) % Ltrack - Ltrack / 2
            s_prev = s_now
            if lap_start is None and progress >= first_line:
                lap_start = t
            elif lap_start is not None and progress >= first_line + (laps_done + 1) * Ltrack:
                res.laps.append(t - lap_start)
                lap_start = t
                laps_done += 1
            if int(t / 0.05) != int((t - cfg.dt) / 0.05):
                res.trace.append((x, y, v))
        return res

    def _hits_opponent(self, x, y, yaw, res: SimResult) -> bool:
        if not self.opponents:
            return False
        me = self.body_corners(x, y, yaw)
        cme = me.mean(axis=0)
        for o in self.opponents:
            ox, oy, oh = self._opp_pose(o)
            res.min_opponent_gap = min(res.min_opponent_gap, math.hypot(cme[0] - ox, cme[1] - oy))
            if _rects_overlap(me, self.body_corners(ox, oy, oh, rear_axle=False)):
                return True
        return False


def _rects_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """Separating axis test for two convex quads."""
    for poly in (a, b):
        for k in range(4):
            e = poly[(k + 1) % 4] - poly[k]
            axis = np.array([-e[1], e[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


# -------------------------------------------------------------------- controllers

class _Base:
    def reset(self, x, y, yaw): ...
    def on_odom(self, wl, wr, gz, dt): ...
    def on_scan(self, angles, ranges): self._scan = (angles, ranges)
    def estimate(self): return None
    def racing(self): return True
    def in_fallback(self): return False


class PPController(_Base):
    """Pure Pursuit on the centreline using ground-truth pose (upper bound, no localization)."""

    def __init__(self, sim: Sim, limits: SpeedLimits | None = None):
        self.path = sim.center
        self.limits = limits or SpeedLimits(zones=bump_zone(sim.scene))
        self.v_ref = speed_profile(self.path, self.limits)
        self.pp = PurePursuit(sim.p)
        self.p = sim.p

    def reset(self, x, y, yaw):
        self.pp.reset(self.path.nearest_index(x, y))

    def command(self, truth):
        x, y, yaw, v = truth
        delta, i = self.pp.steer(self.path, x, y, yaw, max(v, 0.5))
        j = self.path.advance(i, 0.15 + 0.1 * v)
        v_cmd = min(self.v_ref[i], self.v_ref[j], steer_speed_cap(delta, self.p, self.limits.a_lat * 1.2))
        return delta, v_cmd


class FTGController(_Base):
    def __init__(self, sim: Sim, ftg: FollowTheGap | None = None):
        self.ftg = ftg or FollowTheGap(sim.p)
        self._scan = None

    def command(self, truth):
        return self.ftg.command(*self._scan) if self._scan else (0.0, 0.0)


class StackController(_Base):
    """The real race stack fed only with noisy sensors. Truth is used for scoring only."""

    def __init__(self, sim: Sim, cfg: RaceConfig | None = None):
        self.stack = RaceStack(sim.center, sim.map, sim.scene, sim.p, cfg)

    def reset(self, x, y, yaw):
        self.stack.reset_pose(x, y, yaw)
        self.stack.on_start_light(True)

    def on_odom(self, wl, wr, gz, dt):
        self.stack.on_odom(wl, wr, gz, dt)

    def on_scan(self, angles, ranges):
        self.stack.on_scan(angles, ranges)

    def command(self, truth):
        return self.stack.command()

    def estimate(self):
        return self.stack.pose()

    def in_fallback(self):
        return self.stack.mode is Mode.FALLBACK


def plot(sim: Sim, res: SimResult, path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m = sim.grass
    ext = [m.origin[0], m.origin[0] + m.width * m.resolution, m.origin[1], m.origin[1] + m.height * m.resolution]
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(m.image, cmap="gray", extent=ext, vmin=0, vmax=255)
    tr = np.array(res.trace)
    if len(tr):
        sc = ax.scatter(tr[:, 0], tr[:, 1], c=tr[:, 2], s=2, cmap="turbo")
        fig.colorbar(sc, ax=ax, label="speed [m/s]")
    ax.set_title(res.summary(), fontsize=7)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["stack", "pp", "ftg"], default="stack")
    ap.add_argument("--laps", type=int, default=5)
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--a-lat", type=float, default=0.45, help="lateral accel limit in g")
    ap.add_argument("--v-max", type=float, default=3.0)
    ap.add_argument("--opponents", type=int, default=0, help="slower cars ahead (0.8 m/s)")
    ap.add_argument("--lane-change", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    sim = Sim(SimConfig(seed=args.seed))
    for k in range(args.opponents):
        s0 = sim.center.s[sim.center.nearest_index(*sim.start_pose(args.slot)[:2])]
        sim.add_opponent(s0 + 2.0 + 3.0 * k, 0.8)
    lim = SpeedLimits(a_lat=args.a_lat * 9.81, v_max=args.v_max, zones=bump_zone(sim.scene))
    if args.mode == "pp":
        ctrl = PPController(sim, lim)
        print(f"planned lap (profile only): {lap_time(sim.center, ctrl.v_ref):.2f} s")
    elif args.mode == "ftg":
        ctrl = FTGController(sim)
    else:
        ctrl = StackController(sim, RaceConfig(laps=args.laps, limits=lim, lane_change_enabled=args.lane_change))
    res = sim.run(ctrl, laps=args.laps, slot=args.slot)
    print(res.summary())
    if args.plot:
        import os

        os.makedirs(os.path.dirname(args.plot) or ".", exist_ok=True)
        plot(sim, res, args.plot)
        print("plot ->", args.plot)


if __name__ == "__main__":
    main()
