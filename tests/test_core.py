import math

import numpy as np
import pytest

from racing.control import VehicleParams, ediff, steer_speed_cap
from racing.planning import SpeedLimits, bump_zone, lap_time, speed_profile
from racing.start_light import LampReading, StartDetector, StartState, read_lamps
from racing.track import OFFICIAL_DIR, OccupancyMap, load_centerline, load_scene


@pytest.fixture(scope="module")
def center():
    return load_centerline()


def test_centerline_on_free_space(center):
    m = OccupancyMap.load(OFFICIAL_DIR / "map.yaml")
    assert not np.any(m.is_occupied(center.xy[:, 0], center.xy[:, 1]))
    assert center.length == pytest.approx(46.63, abs=0.05)


def test_min_radius_matches_official(center):
    r_min = 1.0 / np.max(np.abs(center.curvature))
    assert 0.25 < r_min < 0.35  # official: 0.299 m


def test_raycast_hits_wall_both_sides(center):
    m = OccupancyMap.load(OFFICIAL_DIR / "map.yaml")
    i = center.index_at_s(5.0)  # start straight
    x, y = center.xy[i]
    h = center.heading[i]
    left, right = m.raycast(x, y, np.array([h + math.pi / 2, h - math.pi / 2]), 3.0)
    # wall inner face = road half width 0.225 + grass 0.20
    assert left == pytest.approx(0.425, abs=0.03)
    assert right == pytest.approx(0.425, abs=0.03)


def test_speed_profile_respects_limits(center):
    lim = SpeedLimits(a_lat=0.45 * 9.81, v_max=3.0, zones=bump_zone(load_scene()))
    v = speed_profile(center, lim)
    assert v.max() <= 3.0 + 1e-9
    assert np.all(v**2 * np.abs(center.curvature) <= lim.a_lat * 1.001 + 1e-9)
    s_bump = load_scene()["speed_bumps"]["bumps"][0]["s_m"]
    assert v[center.index_at_s(s_bump)] <= 1.0 + 1e-9
    assert 18.0 < lap_time(center, v) < 26.0


def test_ediff_at_min_radius():
    p = VehicleParams()
    delta = math.atan(p.wheelbase / 0.3)
    vl, vr = ediff(1.0, delta, p)
    assert vl < 1.0 < vr
    assert (vr - vl) / 2 == pytest.approx(p.track_width / 2 / 0.3, rel=1e-6)  # ~ +-22 %
    assert math.isinf(steer_speed_cap(0.0, p))


# ----------------------------------------------------------------- start light

def _render(reds_on: int, green: bool, size=(120, 260)) -> np.ndarray:
    import cv2

    img = np.full((*size, 3), 30, np.uint8)
    for k in range(5):
        cx, cy = 30 + k * 50, 60
        if k < 4:
            col = (0, 0, 255) if k < reds_on else (20, 20, 60)
        else:
            col = (0, 255, 0) if green else (20, 60, 20)
        cv2.circle(img, (cx, cy), 12, col, -1)
    return img


def _official_sequence(rng):
    """Frames at 30 fps following the Arduino sketch (issue #3)."""
    frames = []
    def hold(reds, green, sec):
        frames.extend([(reds, green)] * int(round(sec * 30)))
    hold(0, False, 1.0)
    hold(1, False, 1.0)
    hold(2, False, 1.0)
    hold(3, False, rng.integers(1000, 3001) / 1000)
    hold(4, False, rng.integers(1000, 3001) / 1000)
    go_frame = len(frames)
    hold(0, True, 5.0)
    return frames, go_frame


def test_read_lamps_counts():
    for n in range(5):
        r = read_lamps(_render(n, False))
        assert r.reds == n and not r.green
    r = read_lamps(_render(0, True))
    assert r.reds == 0 and r.green


@pytest.mark.parametrize("seed", range(5))
def test_detector_fires_on_green_never_early(seed):
    frames, go_frame = _official_sequence(np.random.default_rng(seed))
    det = StartDetector()
    fired = None
    for i, (reds, green) in enumerate(frames):
        if det.update(read_lamps(_render(reds, green))) is StartState.GO and fired is None:
            fired = i
    assert fired is not None
    assert go_frame <= fired <= go_frame + 3  # <= ~100 ms at 30 fps


def test_green_without_reds_is_ignored():
    det = StartDetector()
    for _ in range(30):
        assert det.update(LampReading(reds=0, green=True)) is StartState.WAITING


# ----------------------------------------------------------------- sim smoke

@pytest.mark.slow
def test_sim_pp_and_ftg_complete_a_lap():
    from sim.sim2d import FTGController, PPController, Sim

    sim = Sim()
    res = sim.run(PPController(sim), laps=1)
    assert not res.crashed and len(res.laps) == 1 and res.grass_penalties == 0
    res = Sim().run(FTGController(sim), laps=1)
    assert not res.crashed and len(res.laps) == 1 and res.grass_penalties == 0
