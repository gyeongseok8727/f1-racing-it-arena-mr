# 실차 bring-up 순서

코드는 준비됐고, 여기부터는 **사람이 차를 만지면서** 진행한다. 위에서부터 순서대로. 각 단계의 값은 표시된 파일에 적는다.

## 0. 준비물

- [ ] `firmware/common/secrets.h.example` → `secrets.h` 복사, **PMK/LMK 16바이트를 랜덤으로** 바꾸기 (git에 안 올라감)
- [ ] Jetson SSH 접속 정보 → Claude에게 전달 (IP 또는 Tailscale 주소, 사용자 이름)
- [ ] 구매한 IMU 모델명 → `firmware/vehicle/platformio.ini`의 `-DIMU_MODEL=` (`IMU_MPU6050` / `IMU_LSM6DSOX` / `IMU_NONE`)

## 1. 배선 (전원 끄고)

| 항목 | 확인 |
|---|---|
| K1 핀 | **GPIO14는 부팅 때 PWM이 튄다.** 가능하면 13 또는 23으로 배선하고 `config.h`의 `PIN_K1` 수정 |
| 모터 PWM | PM28 드라이버 입력이 5 V면 레벨 시프터. **PWM 선이 떠 있을 때 정지하는지**(풀업) 확인 → `PWM_STOP_HIGH` |
| FG | 오픈 컬렉터면 3.3 V 풀업. GPIO 34/35는 내부 풀업 없음 |
| 서보 | ST3215 버스 어댑터 (반이중) → Serial2 |
| 정지 버튼 | 차량 탑재, GPIO32 ↔ GND |
| 전원 | PDB J01 → Jetson / M01 pass-through → 모터. **Jetson 먼저, 모터 나중** |

## 2. 펌웨어 (바퀴 띄우고)

```bash
cd firmware/handheld && pio run -t upload && pio device monitor   # MAC 주소 기록
cd firmware/vehicle  && pio run -t upload && pio device monitor   # MAC 주소 기록
```

- [ ] 두 MAC을 `secrets.h`에 넣고 두 보드 다시 업로드
- [ ] 휴대기 ARM → 차량 LED 켜짐, K1 닫힘
- [ ] 휴대기 STOP → 즉시 꺼짐. **다시 ARM 누르기 전까지 안 켜지는지**
- [ ] 휴대기 전원 차단(비상정지 버튼) → 0.2 s 안에 꺼짐
- [ ] 차량 정지 버튼 → 꺼짐
- [ ] Jetson 명령 끊김(USB 뽑기) → 0.15 s 안에 바퀴 정지

## 3. 측정해서 `firmware/vehicle/src/config.h`에 넣을 값

| 값 | 재는 법 |
|---|---|
| `DIR_FORWARD_LEVEL_L/R` | 작은 속도로 돌려 보고 방향 맞추기 |
| `FG_PULSES_PER_WHEEL_REV` | 바퀴에 표시, 손으로 10바퀴 돌리며 FG 펄스 세기 (`/vehicle/status` 대신 펌웨어 로그) |
| `WHEEL_DIAMETER_M` | 바닥에 놓고 5 m 굴린 거리 ÷ 바퀴 회전수 ÷ π (TPU 눌림 포함) |
| `V_FULL_DUTY_MPS` | 바퀴 띄우고 100 % 듀티 속도 (피드포워드) |
| `SERVO_CENTER_TICKS` | 직진이 되는 서보 값 |
| `SERVO_TICKS_PER_RAD`, `SERVO_DIR` | 서보 값 여러 개에서 실제 바퀴 각도 측정 (좌우 따로) |
| `STEER_MAX_RAD` | 최대 조향각. **26° 이상 안 나오면 링크 수정** (반경 0.3 m 코너) |

## 4. Jetson

```bash
git clone https://github.com/gyeongseok8727/f1-racing-it-arena-mr.git && cd f1-racing-it-arena-mr
bash scripts/setup_jetson.sh
```

`ros2_ws/src/mr_racing/config/race.yaml`의 TODO(HW): 시리얼 포트, 라이다 위치(`lidar_x`, `lidar_yaw`), 카메라 위치·각도.

```bash
ros2 launch mr_racing race.launch.py use_race:=false use_vision:=false   # 브리지만
ros2 run mr_racing teleop_check --ros-args -p speed:=0.5 -p duration:=4.0
ros2 topic echo /vehicle/status
```

- [ ] 직진 4 s 후 `/odom` 거리 = 줄자로 잰 거리 (±3 %)
- [ ] 제자리 원 돌기: `/imu` 적분 각도 = 실제 회전

## 5. 테스트베드 (폼 8장)

- [ ] **횡가속 측정**: 원 반경 고정, 속도 올리며 미끄러지는 속도 v → `a_lat = v²/R` → `race.yaml`의 `a_lat_g`
- [ ] 라이다 `lidar_yaw`: 정면 벽까지 `/scan` 0° 거리가 줄자와 맞는지
- [ ] `race.launch.py` 전체 → `ros2 service call /race_node/start std_srvs/srv/Trigger` 로 수동 출발
- [ ] ArUco: `aruco_print_sheet.pdf` **100 % 인쇄**, 1.0/1.5/2.0 m 에서 `/aruco/fix` 나오는지
- [ ] 신호등: 실물(또는 LED 모형) 앞에서 `save_frame:=/tmp/grid.jpg` → `start_roi` 정하기 → `/start_light/state`

## 6. 대회 당일

- [ ] 프리랩 결과로 받은 그리드 슬롯 → `grid_slot`
- [ ] 세이브 포인트 위치에서 재시작 시 rviz "2D Pose Estimate" (`/initialpose`) 또는 `~/reset`
- [ ] 검차: 15 × 20 cm, 높이 35 cm 이하(실질), 비상정지 HW·SW 시연, 후방 마커 ID 1
