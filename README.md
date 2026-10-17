# MR (KAIST) — ISTech IT Arena 자율주행 차량

2026-11-14 POSTECH ISTech IT Arena 출전 차량 개발 저장소. **비공개** — 결산안·차량 스펙은 다른 팀에 공개하지 않습니다.

| 문서 | 내용 |
|---|---|
| [docs/PLAN.md](docs/PLAN.md) | 차량 구현 계획 v10 (구조·위치추정·제어·ESP32·비상정지·일정) |
| [docs/PLAN_REVIEW.md](docs/PLAN_REVIEW.md) | 공식 에셋·ISAAC 비교 검토, 우선순위 조정안 |

공식 규칙·트랙 파일: [MOSW626/istech-it-arena](https://github.com/MOSW626/istech-it-arena) (`MANUAL.md`, `track/`)

## 구조

| 경로 | 내용 |
|---|---|
| `track/official/` | 공식 트랙 파일 사본 (수정 금지, 출처 커밋은 `SOURCE.md`) |
| `racing/` | ROS 없는 핵심 로직 — 맵·경로 로드, 속도 프로파일, Pure Pursuit, e-diff, Follow the Gap, 신호등 검출 |
| `sim/sim2d.py` | 공식 맵 위 2D 시뮬 (기구학 + 라이다 raycast, 벽 충돌·잔디 판정·랩타임) |
| `tests/` | pytest |

## 실행

```bash
python -m venv .venv && .venv/bin/pip install -e ".[dev]"   # Windows: .venv\Scripts\pip
python -m pytest -q                                          # 13 tests, ~25 s
python -m sim.sim2d --mode pp  --laps 5 --plot out/pp.png    # 맵 기반 Pure Pursuit (정답 위치 사용)
python -m sim.sim2d --mode ftg --laps 2 --plot out/ftg.png   # 맵 없는 대비책
```

현재 시뮬 결과 (2026-10-17): Pure Pursuit a_lat 0.45 g 랩당 22.8 s, Follow the Gap 32.6 s, 둘 다 충돌·잔디 0회.

## 일정

| 시점 | 목표 |
|---|---|
| ~10/24 | gym에서 폴백 주행, ESP32 펌웨어 벤치 시험, 신호등 검출 |
| **10/31** | 실차 저속 완주 |
| ~11/7 | 파티클 필터 + 속도 프로파일 + 앞차 추종 |
| **11/10** | 코드 동결 |
| 11/13 | 결산안 제출 |
| 11/14 | 대회 |
