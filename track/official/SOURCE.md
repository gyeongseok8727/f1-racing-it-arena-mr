# 공식 트랙 파일 사본

출처: [MOSW626/istech-it-arena](https://github.com/MOSW626/istech-it-arena) `track/output_final/`
커밋: `b27f94730b869837a804a3088447103d6b35b87e` (릴리스 v2026.10.10 이후 main, 2026-10-17 복사)

이 폴더의 파일은 **수정하지 않는다.** 공식 트랙이 갱신되면 새 커밋에서 통째로 다시 복사하고 위 커밋 해시를 바꾼다.
`world.sdf`·`venue_layout.dxf`는 쓰지 않아 복사하지 않았다.

| 파일 | 내용 |
|---|---|
| `map.png` / `map.yaml` | 점유격자 0.01 m/px. 0 = 벽, 254 = 주행 가능(도로+잔디), 205 = 트랙 바깥 |
| `map_with_grass.png` / `.yaml` | 같은 맵, 잔디만 150 |
| `centerline.csv` | 메인 중심선 `x_m, y_m, s_m, track_width_m`. s 증가 방향이 주행 방향 |
| `left_boundary.csv` / `right_boundary.csv` | 도로 경계 |
| `branch_0.csv` / `branch_1.csv` | 지름길 중심선 (폭 0.20 m) |
| `scene.json` | 그리드 슬롯 pose, ArUco pose, 방지턱, 신호등, 검증 결과 |
| `traffic_light.py` | 실물 신호등 시퀀스 시뮬 (UDP, 실전에는 없음) |
| `aruco/`, `aruco_print_sheet.pdf` | 코스 마커 ID 0/20/30/45 (판 10 cm / 코드 7 cm) |
