# MR (KAIST) — ISTech IT Arena 자율주행 차량

2026-11-14 POSTECH ISTech IT Arena 출전 차량 개발 저장소. **비공개** — 코드·결산안·차량 스펙을 다른 팀에 공개하지 않습니다.

| 문서 | 내용 |
|---|---|
| [docs/PLAN.md](docs/PLAN.md) | 차량 구현 계획 v10 |
| [docs/PLAN_REVIEW.md](docs/PLAN_REVIEW.md) | 공식 에셋·ISAAC 비교 검토, 우선순위 조정안 |
| [docs/BRINGUP.md](docs/BRINGUP.md) | **실차 bring-up 순서 — 사람이 할 일** |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Jetson↔ESP32 UART, ESP-NOW 비상정지 패킷 |

공식 규칙·트랙: [MOSW626/istech-it-arena](https://github.com/MOSW626/istech-it-arena)

## 구조

```
racing/          ROS 없는 핵심 로직 (laptop Python 3.12 / Jetson 3.10 공용)
  track.py         공식 맵·중심선·scene 로드, raycast
  planning.py      곡률·가감속 속도 프로파일, 과속방지턱 감속 구간
  control.py       Pure Pursuit, e-diff, Follow the Gap(맵 없는 대비책)
  localization.py  바퀴+자이로 오도메트리, 스캔 de-skew, 파티클 필터 (likelihood field)
  race.py          레이스 스택: 출발 대기 → 주행 → 5랩, 위치추정 실패 시 대비책 자동 전환,
                   앞차 추종(·선택: 잔디 레인 추월), 바퀴 스케일 온라인 보정
  opponents.py     맵에 없는 스캔 점 → 상대차
  start_light.py   신호등 (빨강 확인 후 초록 점등 = GO, 시간 추정 안 함)
  aruco.py         코스 마커 → 절대 위치 (PF yaw 힌트로 1~2 cm)
  protocol.py      UART 프레임 (펌웨어와 동일)
sim/sim2d.py     공식 맵 위 2D 시뮬 (노이즈 센서, 상대차, 벽·잔디 판정, 랩타임)
firmware/        ESP32 PlatformIO — vehicle(주행 제어+비상정지), handheld(ESP-NOW 송신기)
ros2_ws/         ROS 2 Humble 패키지 mr_racing — esp32_bridge, race_node, vision_node
track/official/  공식 트랙 파일 사본 (수정 금지)
scripts/         setup_jetson.sh
```

## 실행

```bash
python -m venv .venv && .venv/bin/pip install -e ".[dev]"     # Windows: .venv\Scripts\pip
python -m pytest -q                                            # 26 tests, ~70 s
python -m sim.sim2d --mode stack --laps 5 --plot out/stack.png # 실제 스택 (노이즈 센서 + PF)
python -m sim.sim2d --mode stack --opponents 1                 # 느린 앞차 추종
python -m sim.sim2d --mode ftg --laps 2                        # 대비책만

cd firmware/vehicle && pio run                                 # 펌웨어 빌드 (pip install platformio)
```

Jetson: [docs/BRINGUP.md §4](docs/BRINGUP.md).

## 현재 상태 (2026-10-17, 시뮬 기준)

| 항목 | 결과 |
|---|---|
| 레이스 스택 (PF, 노이즈 센서) a_lat 0.45 g | 랩 22.6 s, 충돌·잔디 0, 위치 오차 ≤ 6.7 cm (스케일 보정 후) |
| 대비책 Follow the Gap | 랩 32.6 s, 충돌·잔디 0 |
| 앞차(0.8 m/s) 추종 | 충돌 0, 최소 간격 0.58 m |
| 잔디 레인 추월 (기본 꺼짐) | 추월 성공, 단 옆 간격 4 cm — 실차 검증 전 사용 금지 |
| ArUco (합성 영상) | 1.5 m 이내 1~2 cm |
| 펌웨어 | vehicle(IMU 3종)·handheld 빌드 성공, **실기 미검증** |
| ROS 노드 | 문법 검사만. **Jetson에서 실행 미검증** |

## 일정

| 시점 | 목표 |
|---|---|
| ~10/24 | 펌웨어 업로드·비상정지 검증, 측정값 반영 |
| **10/31** | 실차 저속 완주 (대비책 또는 PF) |
| ~11/7 | PF + 속도 프로파일 + 앞차 추종 실차 |
| **11/10** | 코드 동결 |
| 11/13 | 결산안 제출 |
| 11/14 | 대회 |
