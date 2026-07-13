# SKKU_AutoDrive 구현 로드맵

참고 레포 [SKKUAutoLab/AutoSDV-HPC](https://github.com/SKKUAutoLab/AutoSDV-HPC)를 우리 하드웨어
(아커만 섀시, 전/후 카메라, 후방 라이다, 가변저항+DC모터 조향)에 맞게 이식하는 순서.
파일별 수정 위치·핵심 코드는 [`code_usage_plan.md`](code_usage_plan.md) 참조.

> 플랜 검증 완료 (2026-07-11): code_usage_plan.md의 모든 파일·상수·라인이 실제 레포 코드와 일치함을 대조 확인.

---

## 환경 팩트 (검증됨)

| 항목 | 값 | 비고 |
|---|---|---|
| OS / Python | Ubuntu 24.04 / Python 3.12 | 네이티브 ROS2 = **Jazzy** |
| 레포 기준 | ROS2 **Humble** / Python 3.10 | build/install 산출물이 Humble |
| GPU | RTX 4060 + 드라이버 580 | YOLO `device:=cuda:0` 그대로 사용 가능 |
| 핵심 걸림돌 | `func_lib` 3종이 소스 없이 **Python 3.10 `.pyc`만** 존재 | 3.12에서 `bad magic number`로 로드 불가 |

**결정**: 네이티브 Jazzy + `.pyc` 디컴파일(경로 B). 원본 소스는 확보 불가.
디컴파일 결과 32개 함수 중 29개 온전, 재구현 대상은 실질 1개(`dominant_gradient`).
라이다 `RPLidar` 클래스는 공개 `rplidar` 라이브러리(MIT) vendoring이라 그대로 대체.

---

## Phase 0 — 환경 구축

- [x] pip 의존성 설치 (`~/.local`, `--break-system-packages`): `ultralytics`(+torch 2.13 CUDA, `cuda.is_available()=True`), `scipy`, `matplotlib`, `rplidar-roboticia`
      — 이미 있음: numpy, opencv-python-headless, paho-mqtt, pyserial
- [ ] **ROS2 Jazzy 설치 (apt, sudo 필요 — 사용자 실행 대기)** + `ros-jazzy-cv-bridge` `ros-jazzy-message-filters` `ros-jazzy-tf2-ros`
- [x] `.pyc` → `.py` 복원 3종 **완료** (아래 "복원 상세" 참조). 원본 `.pyc`는 `*.cpython-310.pyc.bak`으로 보존
- [x] `rplidar` pip 설치 (`rplidar-roboticia 0.9.5`)

### 복원 상세 (2026-07-11)

pycdc 디컴파일 + pycdas 바이트코드 대조로 3개 `func_lib` 복원. **주의**: pycdc가 문법에러 없이
**조용히 손상시킨 곳**이 많아(변수를 `None`으로 대체 등), WARNING만 믿으면 안 됨. `None.`/`None[`/
`**(`/`lambda .0`/슬라이스 `[(a:b)]` 패턴 전수 스캔 + 함수별 실행 테스트로 검증 완료.

| 파일 | 바이트코드로 복원한 함수 | 손상 유형 |
|---|---|---|
| `camera_perception_func_lib` | `dominant_gradient` | WARNING(불완전) — 허프변환 전체 로직 재구성 |
| | `get_lane_center` | **조용한 손상** — `None[1:-1]`, `road_target_point_x=None`, `if None<0`, 슬라이스, 시그니처 annotation 누출 |
| | `get_traffic_light_color` | 슬라이스 `[(a:b)]`, `None==` 2곳 |
| | `draw_edge` | kwarg `**(...)` + 컴프리헨션 `lambda .0` |
| | `calculate_slope_between_points` | `None.arctan` → `np.arctan` |
| `decision_making_func_lib` | `calculate_slope_between_points` | `None.arctan` → `np.arctan` (컴파일·import는 통과했으나 런타임 크래시 예정이었음) |
| `lidar_perception_func_lib` | `detect_object` | `range_min<=x<=range_max` 체인비교가 `x<=x`로 손상 + 루프밖 `continue` |
| | `check_consecutive_detections` | `None.current_state` → `not self.current_state` |
| | `RPLidar`/`RPLidarException` | 공개 `rplidar` 라이브러리 재export로 대체 (connect/motor_speed WARNING 회피) |

검증: 3파일 컴파일·import OK, 로직 함수 실행 테스트 통과(detect_object/StabilityDetector/rotate/slope),
cv2 함수 합성이미지 테스트 통과(dominant_gradient=25.0°, get_lane_center=250.0).

## Phase 1 — 빌드/실행 통과 (하드웨어 0) ✅ 완료 (2026-07-12)

- [x] 레포의 `build/` `install/` `log/` 삭제 (Humble 산출물)
- [x] 복원 `.py` 3종을 각 `lib/`에 배치, `.cpython-310.pyc` 제거
- [x] `interfaces_pkg` 커스텀 메시지 빌드
- [x] `colcon build` 통과 — 10개 패키지, 실패 0 (경고만: tests_require/CMake deprecation)
- [x] 목표: 노드가 import 에러 없이 기동

> 빌드는 처음부터 통과했고 **문제는 전부 실행할 때 나왔다** → `RUNTIME_FIXES.md` 참조
> (pyc 로더 / cv_bridge×numpy2 / 영상 경로 / 모델 경로 / motion_planner 크래시)

## Phase 2 — 녹화 데이터로 인지 파이프라인 검증 (하드웨어 0) ✅ 완료 (2026-07-12)

- [x] 레포 샘플(`driving_simulation.mp4`)로
      `image_publisher(video)` → `yolov8` → `lane_info_extractor` → `path_planner` 구동 확인
- [x] 구조/토픽 흐름 검증 — 노드 6개 생존, 죽은 노드 0,
      `/topic_control_signal` 10Hz로 `steering/left_speed/right_speed` 발행 확인
- [x] 프레임 유실 해결 — 커널 UDP 버퍼(208KB) < 이미지(921KB)라 BEST_EFFORT 구독자가 80% 유실.
      sysctl + Fast DDS 프로파일(`fastdds_bigbuf.xml`) **둘 다** 필요. 결과: 6.6Hz → **33.4Hz**,
      `/detections` **34Hz**. 상세 `RUNTIME_FIXES.md` §6

## Phase 3 — 하드웨어 브링업 (장치별 독립)

1. **아두이노** — `check_variable_resistor.ino` 업로드 → 좌우 끝 실측 → `driving.ino`
   `resistance_most_left/right`(460/352)·핀(STEERING 2/3, FORWARD 4~7) 우리 배선에 맞게 → 업로드
2. **시리얼** — `serial_sender_node` `PORT`(`/dev/ttyACM0`, `ls /dev/ttyACM*`) → `s0l0r0` 수동 테스트
3. **카메라** — `data_source:=camera`, `cam_num`(`ls /dev/video*`) → 전방 + 후방(`pub_topic:=image_02, cam_num:=1, name:=...`)
4. **라이다** — `LIDAR_PORT`(`/dev/ttyUSB0`) → §4.2 `offset`으로 후방장착 0도 보정 → §4.3 각도/거리 범위

## Phase 4 — 캘리브레이션 (하드웨어 장착 후)

- [ ] `lane_info_extractor` `src_mat` 4점 우리 카메라(640×480)로 재측정
- [ ] `path_planner` `CAR_CENTER_POINT`(320,179) 실측
- [ ] `lane_width`(300)·`cutting_idx`(250)·`detection_thickness`(10) 트랙 기준 조정
- [ ] `motion_planner` `convert_steeringangle2command(52,...)`의 52, 속도값 80 튜닝

## Phase 5 — 신규 작성 (레포에 없음)

- [x] **신호등** — 로직 완료 (2026-07-13). `best_urp.pt`에 `traffic_light` 클래스가 없어
      YOLO bbox를 못 쓴다 → 영상에서 직접 검출. **실측만 남음** → `docs/CALIBRATION.md` "신호등" 절.

      ★**"색 비율" 방식은 못 쓴다** — 화면 위쪽의 빨강/초록 픽셀 비율로 판정하면
      신호등이 **하나도 없는** 녹화 영상에서 **71%가 오검출**된다 (빨강 55% = 주황 기둥,
      초록 16% = 잔디천·화분). 이 값이 motion_planner로 가면 차가 아무 데서나 선다.

      → **blob 검출로 교체** (`lib/traffic_light_lib.py`, ROS 비의존):
      밝고(V≥180) 진한(S≥120) 픽셀만 남긴 뒤 덩어리를 찾아 **면적·원형도·가로세로비·
      어두운 등기구**로 거른다. 색면(기둥·천)과 점등부의 차이는 색이 아니라 **형태와 밝기**다.
      - 결정타는 **"점등부는 어두운 등기구에 박혀 있다"** (`_has_dark_housing`).
        빨간 자판기·초록 간판은 색도 모양도 램프 같지만 **둘레가 밝아서** 여기서 걸린다.
      - `TrafficLightVoter`: 3프레임 연속 같은 색이어야 확정 (반짝임 방어)

      ⛔**motion_planner의 빨간불 분기도 같이 고쳐야 한다** — 검출부만 바꾸고 두면,
      판단부가 여전히 없는 YOLO bbox(`class_name=='traffic_light'`)를 뒤진다 →
      (1) 진짜 빨간불이어도 **안 멈추고** (2) `elif`엔 들어가서 `else`(정상주행)를 건너뛰어
      **직전 조향값이 그대로 계속 발행**된다(차선추종 정지). 대안 A를 반쪽만 적용한 결과.

      검증 (`python3 scripts/traffic_light_eval.py`, 하드웨어 0):
      오검출 **0%** (71% → 0), 합성 신호등 검출률 **99.9%** (램프 반지름 ≥9px)
- [x] **후진 주차 (직각)** — 로직 완료, `./scripts/test_parking.sh` 3 시나리오 통과 (하드웨어 없이).
- [x] **후진 주차 (직각)** — 로직 완료, `./scripts/test_parking.sh` 3 시나리오 통과 (하드웨어 없이).
      **남은 건 실측뿐** → `docs/CALIBRATION.md` "주차" 절.
      - `lidar_obstacle_detector`: `lidar_min_distance`(Float32) 토픽 추가 (회피용 Bool은 유지)
      - `parking_controller_node` 신규: FSM (IDLE→ALIGN→REVERSE_TURN→REVERSE_STRAIGHT→DONE/ABORT).
        `motion_planner`의 상태 없는 if/elif에 끼워넣지 않고 별도 노드로 분리
      - `motion_planner`: `parking_active`면 양보(relay). **라이다 정지 분기보다 먼저 검사** —
        안 그러면 후진 중 뒷벽이 잡혀서 주차칸 입구에 멈춘다
      - 안전망: 라이다 값 끊기면 즉시 정지 / 전체 제한시간 초과 시 ABORT (개루프 구간 폭주 방지)
      - 트리거는 `parking_trigger`(Bool) 토픽으로 분리 → 지금은 수동 발행,
        나중에 라이다 빈칸 탐지든 YOLO 표지판이든 여기에 꽂기만 하면 FSM은 안 건드림
- [x] **후방 카메라** — `rear_park_detector_node` 신규 (2026-07-13). **주차 FSM의 '눈'.**

      뒷벽이 없다는 게 확인되면서(양옆에 차만 있음) 멈출 근거가 라이다 → 카메라로 바뀌었다.
      `image_02` 구독 → `parking_rear_distance`(뒤 경계선까지 거리) + `parking_lateral_error`
      (칸 중앙에서 좌우 치우침) 발행. FSM은 Float32 하나만 소비하므로 출처를 모른다.

      - **옆차가 뒷선의 좌우를 가린다** → 버드아이뷰의 **중앙 띠**만 훑는다. 우리 차 바로
        뒤는 마지막까지 열려 있다.
      - **안 보이면 침묵한다** — 억지로 추정한 거리를 내보내면 FSM이 믿고 계속 후진한다.
        FSM이 침묵을 상황에 따라 해석한다(아직 못 봄 / 사각지대 = 다 들어옴 / 고장).
      - **'어둡다'를 절대 밝기로 정하지 말 것** — V<110으로 뒀더니 바닥(V=96)까지 차로 잡혀
        화면 전체가 옆차가 됐다. 옆차의 조건은 '어둡다'가 아니라 **'바닥보다 어둡다'** 다.

      검증 (`python3 scripts/rear_park_eval.py`, 합성 장면): 거리오차 최대 0.9cm /
      사각지대에서 침묵 / 치우침 오차 0.0cm. ROS에서도 노드를 띄워 거리 발행 → 사각지대
      진입 시 발행 중단까지 확인.
      **남은 건 실측** — 버드아이뷰 4점(`src_mat`)과 `m_per_px`. 실차 후방 영상이 필요하다.

## Phase 6 — 통합 launch

- [ ] `main.launch.py` 재작성 (§7): `ethernet_image_publisher` 5개 제거 →
      전/후방 `image_publisher` + 주석 처리된 5노드 해제 + 라이다 3노드 추가

---

## 작업 성격 요약

| 성격 | 해당 | 부담 |
|---|---|---|
| 상수/파라미터 교체 | §1~5, §7 대부분 | 낮음 (하드웨어 후 확정) |
| Jazzy 포팅 (빌드 에러 대응) | Phase 1 | 중 (한 번에 몰아서) |
| 코드 신규 작성 | §3.4 신호등, §8 후진주차 | 높음 (실제 개발 집중 지점) |
