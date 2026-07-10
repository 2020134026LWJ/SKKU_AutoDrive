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

## 아두이노 (driving.ino, C — ROS 아님)
- [ ] **가변저항 실측** — `resistance_most_left=460, resistance_most_right=352` → `check_variable_resistor.ino` 업로드해서 좌우 끝값 2개
- [ ] **핀 번호** — `STEERING 2/3, FORWARD 4~7` 실제 배선에 맞게
