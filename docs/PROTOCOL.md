# Jetson ↔ ESP32 UART 프로토콜 / ESP-NOW 비상정지 패킷

구현: Python `racing/protocol.py`, 펌웨어 `firmware/common/protocol.h`. **두 쪽을 항상 같이 고친다.** `tests/test_protocol.py`가 Python 쪽 인코딩을 고정한다.

## UART 프레임

921600 baud 8N1, 리틀 엔디언.

```
A5 5A | type (u8) | len (u8) | payload[len] | crc16 (u16, LE)
```

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), 범위 = `type, len, payload`.

### CMD `0x01` — Jetson → ESP32, 50 Hz

| 필드 | 타입 | 단위 |
|---|---|---|
| seq | u16 | |
| speed | i16 | mm/s (차체 기준, 후륜 평균) |
| steer | i16 | mrad (+ = 좌) |
| flags | u8 | bit0 drive_enable, bit1 ediff_enable |
| ediff_gain | u8 | % (0~100) |

8 bytes. ESP32는 **150 ms** 동안 유효 CMD가 없으면 속도 0 (fault bit0).

### STATE `0x81` — ESP32 → Jetson, 100 Hz

| 필드 | 타입 | 단위 |
|---|---|---|
| seq | u16 | |
| t_ms | u32 | ESP32 부팅 후 ms |
| v_left | i16 | mm/s (FG 측정, 부호는 명령 방향) |
| v_right | i16 | mm/s |
| gyro_z | i16 | mrad/s |
| steer | i16 | mrad (실제 송신한 조향) |
| arm | u8 | 0 DISARMED, 1 ARMED |
| faults | u8 | 아래 표 |
| batt | u16 | mV |
| cmd_seq | u16 | 마지막으로 받은 CMD seq |

20 bytes.

| fault bit | 의미 |
|---|---|
| 0 | Jetson 명령 timeout (150 ms) |
| 1 | 왼쪽 stall (명령 있는데 FG 무응답) |
| 2 | 오른쪽 stall |
| 3 | 비상정지 허가 timeout (200 ms) |
| 4 | 비상정지 STOP 수신 |
| 5 | IMU 오류 |
| 6 | 워치독 리셋 후 부팅 |
| 7 | 최근 1 s 내 CRC 오류 |

## ESP-NOW 허가 패킷 — 휴대기 → 차량

고정 채널, peer MAC unicast, LMK 암호화. 키는 `firmware/common/secrets.h` (git 제외, `secrets.h.example` 참고).

| 필드 | 타입 | |
|---|---|---|
| magic | u16 | `0x524D` ("MR") |
| team | u8 | 팀 ID |
| type | u8 | 1 ARM, 2 PERMIT, 3 STOP |
| session | u32 | ARM 버튼마다 새로 생성 (esp_random) |
| seq | u32 | 패킷마다 +1 |
| crc | u16 | CRC-16/CCITT-FALSE, 앞 12 bytes |

14 bytes, 20 Hz(50 ms) 송신.

상태 규칙 (PLAN.md §6):

- 부팅 = DISARMED. **이전에 쓴 적 없는 session**의 ARM을 받아야 ARMED (K1 닫힘).
- ARMED 중 같은 session의 PERMIT가 **200 ms** 끊기면 DISARMED, 그 session은 폐기.
- STOP, 차량 정지 버튼, stall → DISARMED, session 폐기.
- 폐기된 session의 패킷은 무시 → 신호가 돌아와도 **스스로 다시 출발하지 않는다.**
