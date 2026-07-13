# 하드웨어 캘리브레이션 punch-list

하드웨어 붙이면 바꿔야 하는 값 전부. `file:line` + 현재값(원팀 기준) + 측정법.
값 자체는 하드웨어 없이 확정 불가라 지금은 그대로 두고, 아래를 체크리스트로 사용.

## 시리얼 / 포트 (장치 연결 후 즉시)
- [ ] **아두이노 PORT** — `serial_sender_node.py:17` `PORT='/dev/ttyACM0'` → `ls /dev/ttyACM*`로 확인 (보드 종류 따라 ttyUSB일 수도)
- [ ] **라이다 PORT** — `lidar_publisher_node.py:23` `LIDAR_PORT='/dev/ttyUSB0'` → `ls /dev/ttyUSB*`

## 카메라
- [ ] **입력 소스** — `image_publisher_node.py:21` `DATA_SOURCE='video'`(레포 mp4) → 실행 시 `data_source:='camera'`
- [ ] **카메라 번호** — `image_publisher_node.py:24` `CAM_NUM=0` → `ls /dev/video*`. 후방은 별도 인스턴스(`cam_num:=1, pub_topic:='image_02'`)
- [x] **YOLO device** — `yolov8_node.py:61` `cuda:0` → RTX 4060 확인됨, 그대로 사용 (변경 불필요)

## 차선 인식 (트랙 위에서 카메라로 실측)
- [ ] **원근변환 4점** — `lane_info_extractor_node.py:60` `src_mat=[[238,316],[402,313],[501,476],[155,476]]` → 우리 카메라(640×480) 프레임에서 차선 사각영역 4꼭짓점 재측정 (100% 다시 잡아야 함)
- [ ] **ROI 자르기** — `lane_info_extractor_node.py:63` `cutting_idx=250`
- [ ] **차선 폭/두께** — `lane_info_extractor_node.py:87` `lane_width=300`, `detection_thickness=10`
- [ ] **차량 기준점** — `path_planner_node.py:12` `CAR_CENTER_POINT=(320,179)` → 영상에서 앞범퍼 중심 픽셀 재측정

## 조향 / 속도 (모터 물려서 튜닝)
- [ ] **조향 기준값** — `motion_planner_node.py:108` `convert_steeringangle2command(52, ...)`의 `52`
- [ ] **주행 속도** — `motion_planner_node.py:109-110` `left/right_speed_command = 80` (255가 최대)

## 라이다 (후방 장착 후)
- [ ] **장착방향 보정** — `lidar_processor_node.py:47` `rotate_lidar_data(msg, offset=0)` → 라이다 물리 0도와 우리가 원하는 "정면" 각도차를 `offset`으로 (핵심)
- [ ] (선택) `lidar_processor_node.py:48` `flip_lidar_data(msg, pivot_angle=0)`
- [ ] **감지 각도/거리** — `lidar_obstacle_detector_node.py:43-47` `start_angle=0, end_angle=30, range_min=0.5, range_max=2.0[m]` (offset 맞췄으면 차량기준 그대로)
- [ ] **연속감지 프레임** — `lidar_obstacle_detector_node.py:36` `consec_count=5` (반응속도 vs 오탐)

## 신호등 (traffic_light_detector_node — 전부 ROS 파라미터, 코드 수정 없이 override 가능)

검출 로직은 `lib/traffic_light_lib.py`(ROS 비의존). **차 없이 검증 가능**:
`python3 scripts/traffic_light_eval.py` → 오검출/검출률을 숫자로 뱉는다.
(현재: 녹화 영상 98초에서 **오검출 0**, 합성 신호등 검출률 99.9%)

- [ ] **`min_area` / `max_area`** (기본 60 / 8000) — ★사실상 이 둘만 재면 된다.
      정지선 앞에 차를 세워두고 신호등 램프가 화면에서 **몇 픽셀**로 보이는지 확인 →
      그 면적이 범위 안에 들어오게. 너무 넓게 잡으면 색면(기둥·간판)이 다시 들어온다.
- [ ] **`roi_bottom`** (기본 0.6) — 신호등이 화면 위쪽 몇 %에 걸리나. 노면은 볼 이유가 없다.
- [ ] **`val_min`** (기본 180) — 점등부 밝기 하한. 실내/흐린 날 램프가 어둡게 잡히면 낮춘다.
      낮출수록 오검출이 는다 → 낮추면 `traffic_light_eval.py` 재실행해서 오검출 0 유지 확인.
- [ ] **`consec_frames`** (기본 3) — 반응속도 vs 오탐. 라이다 `consec_count`와 같은 성격.
- [ ] **실물 사진으로 재검증** — 신호등이 찍힌 영상을 하나 녹화해서
      `traffic_light_eval.py`의 `VIDEO`를 그걸로 바꿔 돌리면 **진짜 검출률**이 나온다.
      (지금 검출률은 합성 신호등 기준 — 구조 검증일 뿐 실물 대역은 아니다)

⚠ **`max_area`를 키우거나 `val_min`을 낮출 때는 반드시 `traffic_light_eval.py`를 다시 돌릴 것.**
이 검출기의 전제는 "빨간 물체"가 아니라 "빨갛게 **빛나는** 것"이다. 임계를 풀면 빨간 자판기가
다시 빨간불이 되고, 차는 아무 데서나 선다.

## 주차 (parking_controller_node.py — 전부 ROS 파라미터라 코드 수정 없이 override 가능)

주차 진입 회전은 **개루프**다. 이 차엔 오도메트리도 IMU도 없어서 "90도 돌았는지"를
측정할 방법이 없다. 그래서 아래 duration들이 곧 주차 정확도 그 자체다.

> **뒷벽이 없다** (2026-07-13 확인). 주차장엔 양옆에 차만 있고 뒤는 경계선뿐이다.
> 그래서 최종 접근의 거리 출처가 후방 라이다 → **후방 카메라(뒤 경계선)** 로 바뀌었다.
> FSM은 `parking_rear_distance`(Float32) 하나만 소비하므로 출처를 모른다.
>
> 카메라는 뒷선을 **항상 볼 수 있는 게 아니다.** 이게 주차 로직의 가장 까다로운 부분:
>
> | 구간 | 뒷선 | 이유 |
> |---|---|---|
> | 멀리서 비스듬히 | 안 보임 | **옆차가 가림** |
> | 칸 축과 나란해진 뒤 | 보임 | 폐루프 구간 |
> | 다 들어가서 | 안 보임 | **범퍼 밑 사각지대** |
>
> "안 보임"을 뭉뚱그려 ABORT하면 **정상 주차가 끝날 때마다 중단된다.** FSM은 셋을 가른다:
> 아직 못 봄(SEARCH) / 가까이서 잃음(BLIND_CREEP = 다 들어옴) / 멀리서 잃음(ABORT).

측정 순서대로 적었다. 앞 항목이 틀리면 뒤 항목 측정이 무의미하니 순서대로 할 것.

- [ ] **조향 방향(handedness)** — `slot_side`(기본 `"right"`) / `steer_into_slot()`의 부호.
      차를 띄워놓고 `ros2 topic pub /parking_trigger std_msgs/Bool "{data: true}"` 쏜 뒤
      바퀴가 주차칸 쪽으로 꺾이는지만 본다. **반대로 꺾이면 부호 하나 뒤집으면 끝.**
      → 이거 틀린 채로 아래를 재면 전부 헛수고
- [ ] **모터 데드밴드** — `min_move_speed`(기본 45). PWM을 20→30→40… 올리며 차가 실제로
      구르기 시작하는 값. 이 아래로는 "느리게" 가는 게 아니라 **그냥 안 움직인다**
### 후방 카메라 (rear_park_detector_node) — 주차의 눈. 이걸 먼저 맞춰야 아래가 의미 있다

- [ ] **★버드아이뷰 4점** — `src_mat`(8개 float: x1,y1,x2,y2,x3,y3,x4,y4).
      후방 카메라로 바닥에 **직사각형**(테이프 등)을 찍고, 그 네 꼭짓점의 화면 좌표를 넣는다.
      그러면 그 사각형이 BEV에서 진짜 직사각형이 된다.
- [ ] **★m_per_px** — BEV 세로 1픽셀이 실제 몇 m인가. 위 직사각형의 실제 세로 길이를
      BEV에서의 픽셀 높이로 나눈다.
- [ ] **cam_offset_m** — 렌즈~뒤범퍼 끝 거리. 거리를 '범퍼 기준'으로 만든다
- [ ] **center_band**(기본 0.34) — 옆차 가림을 피해 훑는 중앙 띠 폭. 옆차가 뒷선을 많이
      가리면 좁히고, 뒷선을 놓치면 넓힌다
- [ ] **white_min**(170) / **sat_max**(90) — 흰 선 판정. 바닥이 밝거나 조명이 어두우면 조정
- [ ] **side_darker_than**(0.75) — 옆차 = 바닥 밝기의 이 배율보다 어두운 열.
      절대 밝기가 아니라 **상대값**이라 조명이 바뀌어도 웬만하면 버틴다
- [ ] **실물 영상으로 재검증** — 후방 영상을 하나 녹화해서 `scripts/rear_park_eval.py`의
      장면 합성 대신 그걸 물리면 진짜 오차가 나온다 (지금 수치는 합성 기준 = 구조 검증)

### 주차 FSM

- [ ] **★PWM→속도 환산** — `speed_to_mps`(기본 0.003 [m/s per PWM]).
      **뒷선이 안 보이는 구간의 안전이 전부 이 값에 걸려 있다.** 틀리면 사각지대에서
      너무 적게 가거나(덜 들어감) 너무 많이 간다(선을 넘음).
      재는 법: 후진 PWM을 70으로 고정하고 3초간 굴린 뒤 이동거리 측정 → `거리 / (70 × 3)`
- [ ] **정지 거리** — `parking_stop_dist`(기본 0.10m) / `parking_slow_dist`(기본 0.40m).
      카메라가 재는 건 **뒤 경계선까지의 거리**다. 차 뒤끝 기준으로 환산돼 있는지 확인
- [ ] **★사각지대 문턱** — `blind_enter_dist`(기본 0.30m). 차를 천천히 후진시키며
      `ros2 topic echo /parking_rear_distance`가 **끊기는 순간의 실제 거리**를 잰다.
      [중요] 그 실측값보다 **약간 크게** 잡을 것. 마지막으로 관측되는 값은 문턱보다
      항상 살짝 바깥이라, 똑같이 맞추면 정상 주차를 "멀리서 잃음"으로 오판해 ABORT한다
- [ ] **탐색 예산** — `blind_search_max_dist`(기본 0.50m). 뒷선을 한 번도 못 본 채
      후진할 수 있는 최대 거리. 두 조건 사이에서 잡는다:
      옆차 가림이 풀릴 때까지 갈 거리보다 **길게**, 주차칸 깊이보다 **짧게**
      (짧게를 어기면 칸을 뚫고 나간다)
- [ ] **최종 여유** — `blind_final_gap`(기본 0.05m). 눈감고 마무리한 뒤 뒷선까지 남길 거리
- [ ] **후진 속도** — `reverse_speed`(기본 70)
- [ ] **정렬 전진** — `align_duration`(기본 1.0s) / `align_speed`(기본 60).
      주차칸을 얼마나 지나쳐서 멈출지
- [ ] **★진입 회전 시간** — `turn_duration`(기본 2.5s). 조향 최대로 꺾고 후진했을 때
      차체가 90도 도는 데 걸리는 시간. **주차 정확도를 좌우하는 값.** 다른 값 다 맞춘
      뒤 마지막에 0.5s씩 스윕하며 잡을 것
- [ ] **제한시간** — `parking_timeout`(기본 25s). 위 값들 다 정해진 뒤 실제 소요시간 + 여유

override 예시 (코드 안 건드리고):
```bash
ros2 run decision_making_pkg parking_controller_node --ros-args \
  -p turn_duration:=3.0 -p slot_side:=left -p parking_stop_dist:=0.15
```

## 아두이노 (driving.ino, C — ROS 아님)
- [ ] **가변저항 실측** — `resistance_most_left=460, resistance_most_right=352` → `check_variable_resistor.ino` 업로드해서 좌우 끝값 2개
- [ ] **핀 번호** — `STEERING 2/3, FORWARD 4~7` 실제 배선에 맞게
