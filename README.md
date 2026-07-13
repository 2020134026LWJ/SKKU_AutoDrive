# SKKU_AutoDrive — 성균관대 자율주행 경진대회

아커만 섀시 기반 자율주행 차량. [SKKUAutoLab/AutoSDV-HPC](https://github.com/SKKUAutoLab/AutoSDV-HPC)
(이전 년도 주행 코드)를 우리 하드웨어와 미션에 맞게 가져다 수정하는 프로젝트.

## 대회 미션

```
1) 차선 주행 + 장애물 회피   장애물(차 형태)이 나타나면 옆 차선으로 피했다가 복귀.
                            2~3번 연속, 위치는 랜덤.
2) 신호등                    빨간불 정지 / 초록불 출발
3) 주차                      트랙을 바꿔서. 후진 직각 주차.
```

## 지금 상태 (2026-07-13)

**소프트웨어는 차 없이 검증되는 데까지 끝났다. 남은 건 전부 실측이다.**

| 미션 | 상태 | 검증 |
|---|---|---|
| 차선 주행 | 동작 (원본 코드) | 녹화 영상으로 관통 확인 |
| 장애물 회피 (차선 변경) | 완료 | `avoid_eval.py` 8/8 |
| 신호등 | 완료 | 오검출 71% → 0%, 검출률 99.9% |
| 주차 (후진 직각) | 완료 | 5개 시나리오, 카메라~제어 전 체인 |
| **실측 (캘리브레이션)** | **남음** | 차가 필요 → `docs/CALIBRATION.md` |

## 하드웨어 구성

| 부품 | 연결 | 방식 | 용도 |
|---|---|---|---|
| 전방 카메라 | 노트북(ROS2) | USB | 차선 + 장애물 + 신호등 인식 (YOLO) |
| 후방 카메라 | 노트북(ROS2) | USB | 주차칸 뒤 경계선까지 거리 (주차의 눈) |
| 후방 라이다 | 노트북(ROS2) | USB(`/dev/ttyUSB0`) | 후방 장애물 감지 → 정지 |
| 가변저항 | 아두이노 | A2핀 | 현재 조향각 피드백 |
| 조향 DC모터 | 아두이노 | PWM(2/3핀) | 목표 조향각까지 좌/우 회전 |
| 구동 DC모터 | 아두이노 | PWM(4~7핀) | 노트북이 계산한 좌/우 속도 구동 |
| 아두이노 ↔ 노트북 | — | USB 시리얼 9600bps(`/dev/ttyACM0`) | `s각도l좌속도r우속도` 문자열 |

**보드는 아두이노 메가.** 메가는 2~13번이 전부 PWM이라 위 핀 배치가 그대로 동작한다.
[주의] 우노는 PWM이 3,5,6,9,10,11번뿐이라 4·7번에 속도를 주면 모터가 그냥 꺼진다
(전진 때 왼쪽만, 후진 때 오른쪽만 돌아서 차가 제자리에서 뱅뱅 돈다).

**역할 분담**: 노트북(ROS2)이 인지·판단 전담, 아두이노는 실행 담당(시킨 대로 모터만 돌린다).

## 디렉토리 구조

```
SKKU_AutoDrive/
├── config/
│   └── calibration.yaml        # 실측값 전부 여기. 차에서 잰 숫자는 이 파일만 고친다
├── docs/
│   ├── CALIBRATION.md          # 실측 절차서 (처음 하는 사람용, 0~11단계)
│   ├── ROS2_입문_가이드.md      # ROS 처음이면 이것부터
│   ├── HARDWARE_CODE_MAP.md    # 부품↔코드 지도
│   ├── IMPLEMENTATION.md       # 구현 로드맵 + 미션별 설계
│   ├── RUNTIME_FIXES.md        # 런타임에 터진 문제들 (빌드 통과 ≠ 동작)
│   ├── JAZZY_BUILD_NOTES.md    # Jazzy 빌드 대응
│   └── code_usage_plan.md      # 원본 코드 사용 계획 (파일별 수정 위치)
├── arduino/                    # 아두이노 펌웨어 (.ino) — ROS 아님, IDE로 연다
│   ├── driving/                #   조향+구동 펌웨어 (최종 업로드용)
│   ├── check_variable_resistor/ #  조향 센서 끝값 재는 용
│   └── motor_test/             #   모터 단독 테스트용
├── ros2_ws/src/                # ROS2 워크스페이스
│   ├── camera_perception_pkg/  # 카메라 + YOLO + 차선/신호등/후방 인식
│   ├── lidar_perception_pkg/   # 라이다 + 장애물 판단
│   ├── decision_making_pkg/    # 경로계획 + 회피 + 주차 + 최종 판단
│   ├── serial_communication_pkg/ # ROS2 → 아두이노 시리얼 송신
│   └── launch_pkg/             # 통합 launch
├── models/                     # YOLO 가중치 (git 제외)
└── scripts/                    # 실측 도구 + 차 없이 도는 테스트
```

## 노드 흐름

```
전방 카메라 ─┬→ yolov8 ─┬→ lane_info_extractor → path_planner ─┐
             │          │      (따라갈 차선을 밖에서 바꿀 수 있다)  │
             │          └→ obstacle_avoider ──[target_lane]──────┘
             └→ traffic_light_detector ───────────────────────────┐
                                                                  ↓
후방 카메라 ──→ rear_park_detector ──→ parking_controller ──→ motion_planner ──→ 아두이노
라이다     ──→ lidar_obstacle_detector ───────────────────────────↑
```

**제어 명령을 내는 입은 `motion_planner` 하나뿐이다** (주차 중엔 주차 노드에게 양보).
회피는 조향을 직접 만들지 않고 **"따라갈 차선"만 바꾼다** — 그러면 기존 차선 추종이
알아서 부드럽게 옮겨 간다.

## 실행

```bash
source scripts/setup_env.sh                  # 새 터미널마다 한 번
cd ros2_ws && colcon build --symlink-install
ros2 launch launch_pkg main.launch.py        # config/calibration.yaml 을 자동으로 읽는다

./scripts/stop_all.sh                        # 끝나면 반드시 (안 끄면 배터리가 닳는다)
```

## 차 없이 돌아가는 도구

| 명령 | 하는 일 |
|---|---|
| `python3 scripts/pick_points.py --cam 0` | 버드아이뷰 4점을 화면에서 클릭해서 딴다 |
| `python3 scripts/traffic_light_eval.py` | 신호등 검출기 채점 (오검출/검출률) |
| `python3 scripts/avoid_eval.py` | 장애물 회피 판단 채점 (8개 항목) |
| `python3 scripts/rear_park_eval.py` | 후방 카메라 거리 정확도 채점 |
| `./scripts/test_parking.sh` | 주차 FSM 5개 시나리오 |
| `CAMERA=1 ./scripts/test_parking.sh` | 주차 통합 (카메라→인지→판단→제어 전 체인) |
| `./scripts/stop_all.sh` | 떠 있는 노드 전부 끄기 |

## 실측 (차가 오면 할 일)

**값은 전부 `config/calibration.yaml` 한 파일에 적는다. 코드는 안 건드린다.**

절차는 [`docs/CALIBRATION.md`](docs/CALIBRATION.md) — 0단계부터 순서대로.
앞 단계가 틀리면 뒤 단계는 재봐야 헛수고다.

가장 중요한 셋:

- **조향 방향** (1단계, 5분) — 반대면 차가 주차칸 반대쪽으로 꺾는다.
  컴퓨터로는 확인할 방법이 없어서 차를 띄워놓고 한 번 쏴봐야 한다.
- **PWM→속도 환산** (3단계) — 바퀴 센서가 없어서 "얼마나 갔나"를 이 값으로만 추측한다.
  주차의 사각지대 구간과 회피의 복귀 판단이 전부 여기 걸려 있다.
- **버드아이뷰 4점** (4·6단계) — 카메라마다 다시 잡아야 한다.
  `pick_points.py`로 클릭해서 딴다.

**yaml이 아닌 것 둘**: 아두이노 펌웨어(가변저항 끝값·핀), 장치 포트/카메라 번호.
