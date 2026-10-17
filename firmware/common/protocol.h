// Jetson <-> ESP32 UART framing and ESP-NOW permit packet. Spec: docs/PROTOCOL.md.
// Mirror of racing/protocol.py — change both together (tests/test_stack.py pins the layout).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace proto {

constexpr uint8_t SYNC0 = 0xA5, SYNC1 = 0x5A;
constexpr uint8_t T_CMD = 0x01;
constexpr uint8_t T_STATE = 0x81;
constexpr uint8_t FLAG_DRIVE = 0x01;
constexpr uint8_t FLAG_EDIFF = 0x02;

enum Fault : uint8_t {
  F_JETSON_TIMEOUT = 1 << 0,
  F_STALL_LEFT = 1 << 1,
  F_STALL_RIGHT = 1 << 2,
  F_ESTOP_TIMEOUT = 1 << 3,
  F_ESTOP_STOP = 1 << 4,
  F_IMU = 1 << 5,
  F_WATCHDOG_BOOT = 1 << 6,
  F_CRC_ERRORS = 1 << 7,
};

#pragma pack(push, 1)
struct Cmd {
  uint16_t seq;
  int16_t speed_mm_s;
  int16_t steer_mrad;
  uint8_t flags;
  uint8_t ediff_gain_pct;
};
static_assert(sizeof(Cmd) == 8, "Cmd layout");

struct State {
  uint16_t seq;
  uint32_t t_ms;
  int16_t v_left_mm_s;
  int16_t v_right_mm_s;
  int16_t gyro_z_mrad_s;
  int16_t steer_mrad;
  uint8_t arm;
  uint8_t faults;
  uint16_t batt_mv;
  uint16_t cmd_seq;
};
static_assert(sizeof(State) == 20, "State layout");

constexpr uint16_t ESPNOW_MAGIC = 0x524D;  // "MR"
enum PermitType : uint8_t { P_ARM = 1, P_PERMIT = 2, P_STOP = 3 };
struct Permit {
  uint16_t magic;
  uint8_t team;
  uint8_t type;
  uint32_t session;
  uint32_t seq;
  uint16_t crc;  // over the first 12 bytes
};
static_assert(sizeof(Permit) == 14, "Permit layout");
#pragma pack(pop)

inline uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Writes a full frame into out (needs 6 + len bytes). Returns frame length.
inline size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out) {
  out[0] = SYNC0;
  out[1] = SYNC1;
  out[2] = type;
  out[3] = len;
  memcpy(out + 4, payload, len);
  uint16_t c = crc16(out + 2, 2 + len);
  out[4 + len] = c & 0xFF;
  out[5 + len] = c >> 8;
  return 6 + len;
}

inline bool permit_valid(const Permit& p, uint8_t team) {
  return p.magic == ESPNOW_MAGIC && p.team == team &&
         p.crc == crc16(reinterpret_cast<const uint8_t*>(&p), 12);
}

inline void permit_seal(Permit& p) { p.crc = crc16(reinterpret_cast<const uint8_t*>(&p), 12); }

// Byte-at-a-time parser (same behaviour as racing.protocol.FrameParser).
class Parser {
 public:
  static constexpr uint8_t MAX_LEN = 64;
  uint32_t crc_errors = 0;

  // Returns true when a complete valid frame is available in type()/payload()/len().
  bool push(uint8_t b) {
    switch (st_) {
      case 0: st_ = (b == SYNC0) ? 1 : 0; break;
      case 1: st_ = (b == SYNC1) ? 2 : (b == SYNC0 ? 1 : 0); break;
      case 2: type_ = b; st_ = 3; break;
      case 3:
        len_ = b;
        idx_ = 0;
        st_ = (len_ > MAX_LEN) ? 0 : (len_ ? 4 : 5);
        break;
      case 4:
        buf_[idx_++] = b;
        if (idx_ >= len_) st_ = 5;
        break;
      case 5: crc_ = b; st_ = 6; break;
      case 6: {
        crc_ |= (uint16_t)b << 8;
        st_ = 0;
        uint8_t hdr[2] = {type_, len_};
        uint16_t c = crc16(hdr, 2);
        c = crc16(buf_, len_, c);
        if (c == crc_) return true;
        ++crc_errors;
        break;
      }
    }
    return false;
  }
  uint8_t type() const { return type_; }
  uint8_t len() const { return len_; }
  const uint8_t* payload() const { return buf_; }

 private:
  uint8_t st_ = 0, type_ = 0, len_ = 0, idx_ = 0;
  uint16_t crc_ = 0;
  uint8_t buf_[MAX_LEN];
};

}  // namespace proto
