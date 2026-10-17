"""Lightweight 2D sim on the official occupancy map (no ROS, no Gazebo).

Kinematic bicycle (rear-axle reference) + 2D lidar raycast. Checks: wall contact,
grass entries (>2 wheels on grass = 1 penalty, MANUAL §7), lap times.
Purpose is logic validation, not physics fidelity (PLAN_REVIEW §3: don't sink weeks into sim).

    python -m sim.sim2d --mode pp --laps 5 --plot out/pp.png
    python -m sim.sim2d --mode ftg --laps 2
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from racing.control import FollowTheGap, PurePursuit, VehicleParams, steer_speed_cap
from racing.planning import SpeedLimits, bump_zone, lap_time, speed_profile
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
    seed: int = 0


@dataclass
class SimResult:
    laps: list[float] = field(default_factory=list)
    crashed: bool = False
    crash_time: float | None = None
    grass_penalties: int = 0
    max_speed: float = 0.0
    trace: list[tuple[float, float, float]] = field(default_factory=list)

    def summary(self) -> str:
        laps = ", ".join(f"{t:.2f}" for t in self.laps) or "-"
        state = f"CRASH at {self.crash_time:.2f}s" if self.crashed else "ok"
        return (f"laps [{laps}] s | {state} | grass penalties {self.grass_penalties} "
                f"| vmax {self.max_speed:.2f} m/s")


class Sim:
    def __init__(self, cfg: SimConfig = SimConfig(), params: VehicleParams = VehicleParams()):
        self.cfg, self.p = cfg, params
        self.map = OccupancyMap.load(OFFICIAL_DIR / "map.yaml")
        self.grass = OccupancyMap.load(OFFICIAL_DIR / "map_with_grass.yaml")
        self.center = load_centerline()
        self.scene = load_scene()
        self.rng = np.random.default_rng(cfg.seed)
        self.beam_angles = np.linspace(-math.pi, math.pi, cfg.n_beams, endpoint=False)

    # ---------------------------------------------------------------- geometry
    def footprint(self, x, y, yaw, n=4):
        """Sample points on the body outline (body centred between the axles)."""
        L, W, l = self.p.wheelbase, self.p.width, self.p.length
        cx = L / 2
        xs = np.linspace(cx - l / 2, cx + l / 2, n + 1)
        ys = np.linspace(-W / 2, W / 2, n + 1)
        pts = np.array([(u, v) for u in xs for v in (ys[0], ys[-1])] + [(u, v) for v in ys for u in (xs[0], xs[-1])])
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]

    def wheels(self, x, y, yaw):
        L, t = self.p.wheelbase, self.p.track_width
        pts = np.array([(0, t / 2), (0, -t / 2), (L, t / 2), (L, -t / 2)])
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]

    def scan(self, x, y, yaw):
        lx, ly = x + self.cfg.lidar_x * math.cos(yaw), y + self.cfg.lidar_x * math.sin(yaw)
        r = self.map.raycast(lx, ly, self.beam_angles + yaw, self.cfg.max_range)
        return r + self.rng.normal(0, self.cfg.range_noise, r.shape)

    # ---------------------------------------------------------------- run
    def run(self, controller, laps: int = 5, slot: int = 0) -> SimResult:
        cfg = self.cfg
        x, y, yaw = grid_slot_pose(slot, self.scene)
        # slot pose is the body centre; convert to rear axle
        x -= math.cos(yaw) * self.p.wheelbase / 2
        y -= math.sin(yaw) * self.p.wheelbase / 2
        v, delta = 0.0, 0.0
        cmd_v, cmd_delta = 0.0, 0.0
        res = SimResult()
        Ltrack = self.center.length
        hint = self.center.nearest_index(x, y)
        s_prev = self.center.s[hint]
        progress = 0.0
        first_line = (Ltrack - s_prev) % Ltrack  # distance to the start/finish line
        lap_start, laps_done = None, 0
        on_grass = False
        scan = self.scan(x, y, yaw)
        t = 0.0
        next_ctrl, next_scan = 0.0, cfg.scan_dt
        if hasattr(controller, "reset"):
            controller.reset(x, y, yaw)

        while t < cfg.time_limit and laps_done < laps:
            if t >= next_scan:
                scan = self.scan(x, y, yaw)
                next_scan += cfg.scan_dt
            if t >= next_ctrl:
                cmd_delta, cmd_v = controller.command(t, (x, y, yaw, v), self.beam_angles, scan)
                next_ctrl += cfg.control_dt

            # actuators
            dd = np.clip(cmd_delta - delta, -cfg.steer_rate * cfg.dt, cfg.steer_rate * cfg.dt)
            delta = float(np.clip(delta + dd, -self.p.max_steer, self.p.max_steer))
            dv = np.clip(cmd_v - v, -cfg.d_max * cfg.dt, cfg.a_max * cfg.dt)
            v = max(0.0, v + float(dv))
            # kinematics
            x += v * math.cos(yaw) * cfg.dt
            y += v * math.sin(yaw) * cfg.dt
            yaw += v * math.tan(delta) / self.p.wheelbase * cfg.dt
            t += cfg.dt
            res.max_speed = max(res.max_speed, v)

            fx, fy = self.footprint(x, y, yaw)
            if np.any(self.map.is_occupied(fx, fy)):
                res.crashed, res.crash_time = True, t
                break
            wx, wy = self.wheels(x, y, yaw)
            grass_now = int(np.sum(self.grass.value_at(wx, wy) == PX_GRASS)) > 2
            if grass_now and not on_grass:
                res.grass_penalties += 1
            on_grass = grass_now

            hint = self.center.nearest_index(x, y, hint)
            s_now = self.center.s[hint]
            ds = (s_now - s_prev + Ltrack / 2) % Ltrack - Ltrack / 2
            progress += ds
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


# -------------------------------------------------------------------- controllers

class PPController:
    """Pure Pursuit on the centreline using ground-truth pose (stand-in for localization)."""

    def __init__(self, sim: Sim, limits: SpeedLimits | None = None):
        self.path = sim.center
        limits = limits or SpeedLimits(zones=bump_zone(sim.scene))
        self.v_ref = speed_profile(self.path, limits)
        self.limits = limits
        self.pp = PurePursuit(sim.p)
        self.p = sim.p

    def reset(self, x, y, yaw):
        self.pp.reset(self.path.nearest_index(x, y))

    def command(self, t, state, angles, ranges):
        x, y, yaw, v = state
        # steer from the front axle-ish point for better tracking
        delta, i = self.pp.steer(self.path, x, y, yaw, max(v, 0.5))
        # speed reference a little ahead: covers command/actuator lag
        j = self.path.advance(i, 0.15 + 0.1 * v)
        v_cmd = min(self.v_ref[i], self.v_ref[j], steer_speed_cap(delta, self.p, self.limits.a_lat * 1.2))
        return delta, v_cmd


class FTGController:
    def __init__(self, sim: Sim, ftg: FollowTheGap | None = None):
        self.ftg = ftg or FollowTheGap(sim.p)

    def command(self, t, state, angles, ranges):
        return self.ftg.command(angles, ranges)


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
    ax.set_title(res.summary(), fontsize=8)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["pp", "ftg"], default="pp")
    ap.add_argument("--laps", type=int, default=5)
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--a-lat", type=float, default=0.45, help="lateral accel limit in g (pp)")
    ap.add_argument("--v-max", type=float, default=3.0)
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    sim = Sim()
    if args.mode == "pp":
        lim = SpeedLimits(a_lat=args.a_lat * 9.81, v_max=args.v_max, zones=bump_zone(sim.scene))
        ctrl = PPController(sim, lim)
        print(f"planned lap (profile only): {lap_time(sim.center, ctrl.v_ref):.2f} s")
    else:
        ctrl = FTGController(sim)
    res = sim.run(ctrl, laps=args.laps, slot=args.slot)
    print(res.summary())
    if args.plot:
        import os

        os.makedirs(os.path.dirname(args.plot) or ".", exist_ok=True)
        plot(sim, res, args.plot)
        print("plot ->", args.plot)


if __name__ == "__main__":
    main()
