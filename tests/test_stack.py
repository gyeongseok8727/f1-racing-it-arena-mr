import math

import numpy as np
import pytest

from racing.aruco import ArucoLocalizer, CameraModel, marker_world_T, render_marker_view
from racing.localization import Odometry, deskew, relative_motion
from racing.opponents import follow_speed_cap
from racing.protocol import (
    ESPNOW_ARM, Command, FrameParser, State, T_CMD, T_STATE, crc16_ccitt, espnow_packet,
)
from racing.track import OFFICIAL_DIR, load_centerline, load_scene


# ----------------------------------------------------------------- protocol

def test_crc_reference_value():
    assert crc16_ccitt(b"123456789") == 0x29B1  # CRC-16/CCITT-FALSE check value


def test_frames_roundtrip_through_noisy_stream():
    cmd = Command(seq=7, speed=1.234, steer=-0.2, ediff_gain=0.6).encode()
    st = State(3, 123456, 1.0, 1.1, -0.5, 0.1, True, 0b101, 22.4, 7)
    stream = b"\x00\xff\xa5" + cmd + b"\xa5\x5a\x01\x08garbage" + st.encode()
    p = FrameParser()
    frames = []
    for k in range(0, len(stream), 5):  # split reads
        frames += p.feed(stream[k:k + 5])
    types = [f[0] for f in frames]
    assert T_CMD in types and T_STATE in types
    got = State.decode(dict(frames)[T_STATE])
    assert got.v_right == pytest.approx(1.1) and got.armed and got.cmd_seq == 7
    assert got.fault_names() == ["jetson_timeout", "stall_right"]


def test_cmd_payload_layout_is_stable():
    # firmware/common/protocol.h relies on this exact layout
    f = Command(seq=1, speed=1.0, steer=0.5, drive_enable=True, ediff_enable=False, ediff_gain=1.0).encode()
    assert f.hex() == "a55a0108" + "0100" + "e803" + "f401" + "01" + "64" + f[-2:].hex()


def test_espnow_packet_size():
    assert len(espnow_packet(ESPNOW_ARM, 0xDEADBEEF, 1)) == 14


# ----------------------------------------------------------------- localization helpers

def test_odometry_circle_closes():
    o = Odometry()
    R, v = 0.5, 1.0
    for _ in range(int(2 * math.pi * R / v / 0.01)):
        o.update(v, v, v / R, 0.01)
    assert math.hypot(o.x, o.y) < 0.02


def test_relative_motion_inverse():
    a, b = (1.0, 2.0, 0.5), (1.3, 2.4, 0.9)
    dx, dy, dyaw = relative_motion(a, b)
    c, s = math.cos(a[2]), math.sin(a[2])
    assert (a[0] + c * dx - s * dy, a[1] + s * dx + c * dy) == pytest.approx(b[:2])
    assert dyaw == pytest.approx(0.4)


def test_deskew_straight_motion():
    # wall 1 m ahead; beam taken 0.1 s before scan end while moving 2 m/s -> it read 1.2 m
    ang, rng = deskew(np.array([0.0]), np.array([1.2]), np.array([-0.1]), 2.0, 0.0)
    assert rng[0] == pytest.approx(1.0) and ang[0] == pytest.approx(0.0)


def test_follow_cap_monotonic():
    caps = [follow_speed_cap(g) for g in (0.3, 0.5, 1.0, 2.0)]
    assert caps[0] == 0.0 and caps == sorted(caps)


def test_scan_arrays_mounting_and_timing():
    from racing.scan import scan_arrays

    a, r, dt = scan_arrays(-math.pi, math.pi / 2, [1, 2, 3, float("nan")], yaw_offset=math.pi / 2,
                           time_increment=0.025, scan_time=0.1)
    assert a == pytest.approx([-math.pi / 2, 0.0, math.pi / 2, -math.pi], abs=1e-9)
    assert math.isinf(r[3])
    assert dt == pytest.approx([-0.1, -0.075, -0.05, -0.025])


# ----------------------------------------------------------------- aruco

@pytest.mark.parametrize("mid,dist", [(0, 1.2), (20, 1.5), (45, 1.0)])
def test_aruco_pose_recovery(mid, dist):
    import cv2

    scene = load_scene()
    cam = CameraModel.from_fov(848, 480, 69.4, z=0.15, pitch_up=math.radians(12))
    loc = ArucoLocalizer(scene, cam)
    m = next(m for m in scene["aruco_markers"]["markers"] if m["id"] == mid)
    nyaw = m["pose"]["yaw_rad"]
    # car approaching the marker face-on, `dist` m in front of it, slightly offset
    bx = m["pose"]["x"] + math.cos(nyaw) * dist + 0.03 * -math.sin(nyaw)
    by = m["pose"]["y"] + math.sin(nyaw) * dist + 0.03 * math.cos(nyaw)
    byaw = nyaw + math.pi + 0.05
    T = np.eye(4)
    T[:3, :3] = [[math.cos(byaw), -math.sin(byaw), 0], [math.sin(byaw), math.cos(byaw), 0], [0, 0, 1]]
    T[:3, 3] = (bx - cam.x * math.cos(byaw), by - cam.x * math.sin(byaw), 0.0)
    png = cv2.imread(str(OFFICIAL_DIR / "aruco" / f"aruco_id{mid}.png"), cv2.IMREAD_GRAYSCALE)
    img = render_marker_view(png, loc.world[mid], T, cam)
    # full PnP: detected, but rotation from a ~35 px marker is coarse
    fixes = loc.fixes(img)
    assert len(fixes) == 1 and fixes[0].marker_id == mid
    assert math.hypot(fixes[0].x - T[0, 3], fixes[0].y - T[1, 3]) < 0.20
    # with a yaw hint 1 deg off (PF quality) the position is good
    f = loc.fixes(img, yaw_hint=byaw + math.radians(1))[0]
    assert math.hypot(f.x - T[0, 3], f.y - T[1, 3]) < 0.04


# ----------------------------------------------------------------- closed loop

@pytest.mark.slow
def test_stack_laps_with_noisy_sensors_and_follows_opponent():
    from racing.race import RaceConfig
    from racing.planning import SpeedLimits, bump_zone
    from sim.sim2d import Sim, StackController

    sim = Sim()
    res = sim.run(StackController(sim), laps=1)
    assert not res.crashed and len(res.laps) == 1 and res.grass_penalties == 0
    assert res.max_loc_error < 0.2

    sim = Sim()
    s0 = sim.center.s[sim.center.nearest_index(*sim.start_pose(0)[:2])]
    sim.add_opponent(s0 + 2.0, 0.8)
    res = sim.run(StackController(sim, RaceConfig(laps=1, limits=SpeedLimits(zones=bump_zone(sim.scene)))), laps=1)
    assert not res.crashed and len(res.laps) == 1
    assert res.min_opponent_gap > 0.35
