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

## Phase 1 — 빌드/실행 통과 (하드웨어 0)

- [ ] 레포의 `build/` `install/` `log/` 삭제 (Humble 산출물)
- [ ] 복원 `.py` 3종을 각 `lib/`에 배치, `.cpython-310.pyc` 제거
- [ ] `interfaces_pkg` 커스텀 메시지 빌드
- [ ] `colcon build` 통과 (Jazzy API 차이 발생 시 여기서 수정)
- [ ] 목표: 노드가 import 에러 없이 기동

## Phase 2 — 녹화 데이터로 인지 파이프라인 검증 (하드웨어 0)

- [ ] 레포 샘플(`driving_simulation.mp4`, `rosbag2_...`)로
      `image_publisher(video)` → `yolov8` → `lane_info_extractor` → `path_planner` 구동 확인
- [ ] 구조/토픽 흐름 검증 (좌표는 원팀 값 상태라 결과 부정확해도 무방)

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

- [ ] **신호등** — `best_urp.pt`에 `traffic_light` 클래스 없음. §3.4 대안 A:
      YOLO 우회, 전체 프레임에 HSV `cv2.inRange`로 빨강/초록 contour 직접 검출.
      `get_traffic_light_color`의 HSV 범위는 재사용, `detection.bbox` 의존만 제거
- [ ] **후진 주차** — §8. `lidar_obstacle_detector`에 `lidar_min_distance`(Float32) 토픽 추가 +
      `motion_planner`에 `parking_mode` 분기(거리 기반 비례 정지). 주차 트리거/후진 조향/완료 판단은 신규 설계
- [ ] **후방 카메라** — 활용 노드 신규 작성 (모니터링/후방 인식)

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
