"""Jetson <-> ESP32 UART framing (docs/PROTOCOL.md). Mirror of firmware/common/protocol.h."""
from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC = b"\xA5\x5A"
T_CMD = 0x01
T_STATE = 0x81

CMD_FMT = "<HhhBB"  # seq, speed_mm_s, steer_mrad, flags, ediff_gain_pct
STATE_FMT = "<HIhhhhBBHH"  # seq, t_ms, vl, vr, gyro_mrad_s, steer_mrad, arm, faults, batt_mv, cmd_seq

FLAG_DRIVE = 0x01
FLAG_EDIFF = 0x02

FAULTS = ("jetson_timeout", "stall_left", "stall_right", "estop_timeout",
          "estop_stop", "imu", "watchdog_boot", "crc_errors")

ESPNOW_FMT = "<HBBIIH"
ESPNOW_MAGIC = 0x524D
ESPNOW_ARM, ESPNOW_PERMIT, ESPNOW_STOP = 1, 2, 3


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


def _clamp_i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


@dataclass
class Command:
    seq: int
    speed: float  # m/s
    steer: float  # rad
    drive_enable: bool = True
    ediff_enable: bool = True
    ediff_gain: float = 0.7

    def encode(self) -> bytes:
        flags = (FLAG_DRIVE if self.drive_enable else 0) | (FLAG_EDIFF if self.ediff_enable else 0)
        payload = struct.pack(CMD_FMT, self.seq & 0xFFFF, _clamp_i16(self.speed * 1000),
                              _clamp_i16(self.steer * 1000), flags,
                              max(0, min(100, int(round(self.ediff_gain * 100)))))
        return frame(T_CMD, payload)


@dataclass
class State:
    seq: int
    t_ms: int
    v_left: float  # m/s
    v_right: float
    gyro_z: float  # rad/s
    steer: float  # rad
    armed: bool
    faults: int
    batt_v: float
    cmd_seq: int

    @classmethod
    def decode(cls, payload: bytes) -> "State":
        s, t, vl, vr, gz, st, arm, faults, mv, cs = struct.unpack(STATE_FMT, payload)
        return cls(s, t, vl / 1000, vr / 1000, gz / 1000, st / 1000, bool(arm), faults, mv / 1000, cs)

    def encode(self) -> bytes:  # used by tests / a fake ESP32
        payload = struct.pack(STATE_FMT, self.seq & 0xFFFF, self.t_ms & 0xFFFFFFFF,
                              _clamp_i16(self.v_left * 1000), _clamp_i16(self.v_right * 1000),
                              _clamp_i16(self.gyro_z * 1000), _clamp_i16(self.steer * 1000),
                              int(self.armed), self.faults & 0xFF, int(self.batt_v * 1000) & 0xFFFF,
                              self.cmd_seq & 0xFFFF)
        return frame(T_STATE, payload)

    def fault_names(self) -> list[str]:
        return [n for i, n in enumerate(FAULTS) if self.faults & (1 << i)]


class FrameParser:
    """Byte-stream parser tolerant to garbage, split reads and CRC errors."""

    MAX_LEN = 64

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]  # keep a possible half sync byte
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ftype, ln = self.buf[2], self.buf[3]
            if ln > self.MAX_LEN:
                del self.buf[:2]
                continue
            total = 4 + ln + 2
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:4 + ln])
            (crc,) = struct.unpack_from("<H", self.buf, 4 + ln)
            if crc == crc16_ccitt(body):
                out.append((ftype, body[2:]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[:2]


def espnow_packet(ptype: int, session: int, seq: int, team: int = 1) -> bytes:
    head = struct.pack("<HBBII", ESPNOW_MAGIC, team, ptype, session & 0xFFFFFFFF, seq & 0xFFFFFFFF)
    return head + struct.pack("<H", crc16_ccitt(head))
