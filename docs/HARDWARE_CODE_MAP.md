# 하드웨어 ↔ 코드 지도

부품 하나하나가 **어느 코드로, 어떻게 도는지 / 우리는 어떻게 쓸지 / 그래서 뭘 바꿔야 하는지**.
ROS 구조 자체가 헷갈리면 먼저 [`ROS2_입문_가이드.md`](ROS2_입문_가이드.md), 실측값 체크리스트는 [`CALIBRATION.md`](CALIBRATION.md).

---

## 0. 전체 그림 — 두 개의 세계

```
┌─────────────────────── 노트북 (ROS2 / 파이썬) ───────────────────────┐
│                                                                      │
│  [전방카메라]─USB─▶ image_publisher ─📻image_01─▶ yolov8 ─📻detections─┐│
│                                                                     ││
│   ┌─────────────────────────────────────────────────────────────────┘│
│   ├─▶ lane_info_extractor ─📻yolov8_lane_info─▶ path_planner ─┐        │
│   └─▶ traffic_light_detector ─📻...traffic_light_info─┐       │        │
│                                                       ▼       ▼        │
│  [후방라이다]─USB─▶ lidar_publisher ─▶ processor ─▶ obstacle ─📻──▶[motion_planner]│
│                                                               │       │
│                                              📻topic_control_signal    │
│                                                               ▼       │
│                                                      [serial_sender]   │
└───────────────────────────────────────────────────────────────┼──────┘
                                                                 │ USB 시리얼
                                                    "s3l80r80\n"  │ (문자열)
                                                                 ▼
┌─────────────────────── 아두이노 (C / driving.ino) ───────────────────┐
│  문자열 파싱 → 조향 모터 + 좌/우 구동 모터 (가변저항으로 조향 확인)      │
└──────────────────────────────────────────────────────────────────────┘
```

**핵심**: 노트북(파이썬)이 전부 판단하고, 결과 숫자 3개(`조향/좌속도/우속도`)를 문자열로 아두이노(C)에 USB로 넘긴다. 아두이노는 그 숫자대로 모터만 돌린다. (자세히 → 입문가이드 §6)

---

## 1. 전방 카메라

| | |
|---|---|
| **역할** | 차선 인식용 영상 입력 (주행의 핵심 눈) |
| **연결** | 노트북 USB |
| **다루는 코드** | `camera_perception_pkg/image_publisher_node.py` (영상 발행) |

**데이터 흐름**: 카메라 → `image_publisher_node`가 프레임을 잡아 `📻 image_01`(Image)로 발행 → `yolov8_node`가 받아 차선/사물 검출 → `📻 detections` → `lane_info_extractor_node`가 차선 중앙점 추출 → `path_planner_node`가 경로 계산.

**우리 사용 계획**: 640×480, 전방 장착. 원팀은 저장된 mp4로 테스트했지만 우리는 실제 웹캠.

**바꿀 코드**:
- `image_publisher_node.py:21` `DATA_SOURCE='video'` → 실행 시 `data_source:='camera'`, `cam_num:=0`(`ls /dev/video*`로 번호 확인). launch에는 이미 반영해둠(`main.launch.py`).
- `lane_info_extractor_node.py:60` `src_mat` 4점 → **우리 카메라로 재측정 필수** (원근변환 기준, 100% 다시 잡아야 함).
- `path_planner_node.py:12` `CAR_CENTER_POINT=(320,179)` → 영상에서 앞범퍼 중심 픽셀 재측정.

---

## 2. 후방 카메라

| | |
|---|---|
| **역할** | 후방 인식 (현재 레포엔 활용 노드 **없음**) |
| **연결** | 노트북 USB |
| **다루는 코드** | 없음 — `image_publisher_node`를 재활용해 영상만 발행 가능 |

**데이터 흐름**: `image_publisher_node`를 두 번째 인스턴스로 띄워 `📻 image_02`로 발행까지는 됨. 그걸 **소비하는 노드(후방 인식/주차)는 우리가 새로 짜야 함.**

**우리 사용 계획**: 카메라 기반 후진 주차에 쓸 예정 (당신이 설계·구현).

**바꿀 코드**:
- `main.launch.py`에 후방 인스턴스 이미 추가함 (`cam_num:=1, pub_topic:='image_02', name:='image_publisher_rear'`).
- **신규 작성 필요**: `image_02`를 구독해 주차 지점을 찾는 노드 (당신 몫).

---

## 3. 후방 라이다

| | |
|---|---|
| **역할** | 후방 장애물 감지 → 감지되면 정지 |
| **연결** | 노트북 USB (시리얼 `/dev/ttyUSB0`) |
| **다루는 코드** | `lidar_perception_pkg/`의 노드 3개 |

**데이터 흐름 (3단 파이프라인)**:
1. `lidar_publisher_node` — RPLidar에서 스캔 읽어 `📻 lidar_raw`(LaserScan) 발행. (RPLidar 클래스 = 공개 `rplidar` 라이브러리, 우리가 복원 시 이걸로 대체함)
2. `lidar_processor_node` — 장착방향 보정(`rotate_lidar_data(offset)`, `flip_lidar_data`) → `📻 lidar_processed`
3. `lidar_obstacle_detector_node` — 특정 각도·거리 범위에 물체 있나 판정 + `StabilityDetector`(연속 감지 필터) → `📻 lidar_obstacle_info`(Bool) → `motion_planner`가 True면 정지

**우리 사용 계획**: **후방** 장착 (원팀은 다른 방향 가정). 그래서 `offset` 보정이 핵심. 단, 이 로직은 "감지=정지"라 후진 주차(장애물에 다가가야 함)엔 그대로 못 씀 → 주차용 거리 로직은 별도 필요(플랜 §8).

**바꿀 코드**:
- `lidar_publisher_node.py:23` `LIDAR_PORT='/dev/ttyUSB0'` → `ls /dev/ttyUSB*` 확인.
- `lidar_processor_node.py:47` `rotate_lidar_data(msg, offset=0)` → **후방 장착 각도차를 offset으로** (핵심 보정).
- `lidar_obstacle_detector_node.py:43-47` `start_angle/end_angle/range_min/range_max` → 감지 원하는 각도·거리로.
- `:36` `consec_count=5` → 반응속도 vs 오탐 조정.

---

## 4. 가변저항 (조향각 피드백)

| | |
|---|---|
| **역할** | 지금 바퀴가 얼마나 꺾여있나 **측정** (조향 위치 센서) |
| **연결** | 아두이노 아날로그 핀 `A2` |
| **다루는 코드** | `control/driving/driving.ino` (C, ROS 아님) |

**데이터 흐름**: `analogRead(POT)`로 저항값 읽음 → `map()`으로 -7~7 단계로 변환 → 목표각(`angle`, 노트북이 준 값)과 비교 → 다르면 조향 모터를 그 방향으로 돌림. **즉 "목표 조향각에 도달했는지"를 이 값으로 판단.**

```cpp
resistance = analogRead(POT);
mapped_resistance = map(resistance, resistance_most_left, resistance_most_right, -7, 8);
if (mapped_resistance == angle) maintainSteering();  // 도달 → 멈춤
else if (mapped_resistance > angle) steerLeft();
else steerRight();
```

**우리 사용 계획**: 우리 가변저항의 좌/우 끝값이 원팀과 다름 → 실측 교체 필수.

**바꿀 코드**:
- `driving.ino:17-18` `resistance_most_left=460`, `resistance_most_right=352` → **`check_variable_resistor.ino` 업로드해서 좌/우 끝까지 돌렸을 때 값 2개로 교체.**

---

## 5. 조향 DC모터

| | |
|---|---|
| **역할** | 앞바퀴 좌/우로 꺾기 |
| **연결** | 아두이노 PWM 핀 `2`(STEERING_1), `3`(STEERING_2) |
| **다루는 코드** | `driving.ino` `steerLeft/steerRight/maintainSteering` |

**데이터 흐름**: 목표각 ≠ 현재각이면 고정 속도(`STEERING_SPEED=128`)로 한 방향 회전, 같아지면 정지. **비례제어 아님(bang-bang)** — 목표 근처에서 감속 안 하고 딱 멈춤.

```cpp
void steerLeft()  { analogWrite(STEERING_1, LOW);  analogWrite(STEERING_2, 128); }
void steerRight() { analogWrite(STEERING_1, 128);  analogWrite(STEERING_2, LOW); }
```

**우리 사용 계획**: 그대로 사용 가능. (원하면 비례제어로 개선 가능 — 목표각 근처에서 부드럽게)

**바꿀 코드**:
- `driving.ino:5-6` 핀번호(`STEERING_1=2, STEERING_2=3`) → 실제 배선에 맞게.
- **좌/우 방향(handedness) 확인**: `steerLeft`가 진짜 왼쪽인지 실물에서 확인 (반대면 두 함수 swap).
- (선택) `STEERING_SPEED=128` 고정값을 `abs(목표-현재)`에 비례하게 바꾸면 비례제어.

---

## 6. 좌/우 구동 DC모터

| | |
|---|---|
| **역할** | 전/후진 주행 (좌우 각각) |
| **연결** | 아두이노 PWM `4,5`(우), `6,7`(좌) |
| **다루는 코드** | `driving.ino` `setLeftMotorSpeed/setRightMotorSpeed` |

**데이터 흐름**: 노트북이 준 속도값(`left_speed`, `right_speed`)을 그대로 PWM으로. **부호가 방향**: 양수=전진, 음수=후진. (피드백 없음 — 시킨 대로만)

```cpp
void setLeftMotorSpeed(int speed) {
    if (speed > 0) { analogWrite(FORWARD_LEFT_1, speed); analogWrite(FORWARD_LEFT_2, LOW); }
    else           { analogWrite(FORWARD_LEFT_1, LOW);   analogWrite(FORWARD_LEFT_2, -speed); }  // 음수=후진
}
```

**우리 사용 계획**: 주행은 그대로. **후진 주차 시 음수 속도**를 쓰면 됨 → 이 코드가 이미 후진(음수) 지원하니 재사용 가능.

**바꿀 코드**:
- `driving.ino:7-10` 핀번호(`FORWARD_RIGHT 4/5, FORWARD_LEFT 6/7`) → 실제 배선.
- **전/후진·좌우 방향 실물 확인** (반대면 핀 쌍 swap 또는 부호 반전).
- 속도값 자체(`80`)는 노트북 쪽 `motion_planner_node.py:109-110`에서 정함.

---

## 7. 아두이노 (통합 실행 컨트롤러)

| | |
|---|---|
| **역할** | 노트북 명령을 받아 3개 모터 실행 + 조향 피드백. **판단 안 함.** |
| **연결** | 노트북 USB 시리얼 (`/dev/ttyACM0`, 9600bps) |
| **다루는 코드** | `driving.ino` 전체 |

**데이터 흐름**: 노트북 → `"s{조향}l{좌속도}r{우속도}\n"` 문자열 수신 → 파싱 → 조향각 ±7 클램프 → 50ms마다 모터 제어. 판단·경로계산은 전혀 안 하고 받은 숫자대로만.

**우리 사용 계획**: STM32가 아니라 아두이노 사용 (원팀과 동일 구조). 초음파 미사용.

**바꿀 코드**: 위 4·5·6의 핀/저항값 외엔 로직 수정 불필요.

---

## 8. 노트북 (ROS2 두뇌) + 시리얼 다리

| | |
|---|---|
| **역할** | 카메라·라이다로 인지·판단, 조향/속도 계산, 아두이노로 송신 |
| **다루는 코드** | 파이썬 노드 전부 + `serial_sender_node.py`(다리) |

**데이터 흐름 (판단→송신)**: `motion_planner_node`가 경로+신호등+라이다를 종합해 `📻 topic_control_signal`(MotionCommand: 조향/좌속도/우속도) 발행 → `serial_sender_node`가 이를 `"s..l..r..\n"` 문자열로 만들어 USB로 아두이노에 write.

우선순위: **라이다 장애물 > 빨간 신호등 > 차선 추종** (motion_planner의 if/elif/else).

**바꿀 코드**:
- `serial_sender_node.py:17` `PORT='/dev/ttyACM0'` → `ls /dev/ttyACM*`. ⚠️ 이 노드는 import 시 포트를 열어서 아두이노 없으면 크래시 → `perception.launch.py`엔 제외해둠.
- `motion_planner_node.py:108` 조향 기준값 `52`, `:109-110` 속도 `80` → 실측 튜닝. (죽은코드/NameError는 이미 수정)

---

## 9. 우리가 새로 만들 것 (레포에 없음)

| 기능 | 왜 필요 | 관련 하드웨어 |
|---|---|---|
| **신호등 인식** | 모델(best_urp.pt)에 traffic_light 클래스 없음 → HSV 직접검출로 재작성함 ✅ | 전방 카메라 |
| **카메라 기반 후진 주차** | 레포에 주차 로직 자체가 없음. 당신이 설계·구현 예정 | 후방 카메라 (+ 필요시 라이다 거리) |
| **후방 카메라 활용 노드** | image_02 발행까지만 됨, 소비 노드 없음 | 후방 카메라 |

> 주차를 **카메라 기반**으로 가면 플랜(`code_usage_plan.md` §8)의 라이다 거리 방식과 접근이 다름. 후방 카메라로 주차선/공간을 인식 → 후진(음수 속도)+조향 제어. 설계는 당신이, 구조 붙이는 건 같이.

---

## 10. "바꿀 코드" 한눈 요약 (실측값은 CALIBRATION.md)

| 부품 | 파일:줄 | 바꿀 것 |
|---|---|---|
| 전방카메라 | `image_publisher_node.py:21,24` | data_source→camera, cam_num |
| 차선 | `lane_info_extractor_node.py:60` | src_mat 4점 재측정 |
| 차량기준 | `path_planner_node.py:12` | CAR_CENTER_POINT |
| 라이다 포트 | `lidar_publisher_node.py:23` | LIDAR_PORT |
| 라이다 방향 | `lidar_processor_node.py:47` | offset (후방보정) |
| 라이다 판정 | `lidar_obstacle_detector_node.py:43-47` | 각도/거리 범위 |
| 시리얼 | `serial_sender_node.py:17` | PORT |
| 조향/속도 | `motion_planner_node.py:108-110` | 52, 80 |
| 가변저항 | `driving.ino:17-18` | 좌/우 끝값 |
| 모터 핀 | `driving.ino:5-10` | 핀번호 + 방향 |
