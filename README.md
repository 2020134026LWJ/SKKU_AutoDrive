# SKKU_AutoDrive — 성균관대 자율주행 경진대회

아커만 섀시 기반 자율주행 차량. [SKKUAutoLab/AutoSDV-HPC](https://github.com/SKKUAutoLab/AutoSDV-HPC)
(이전 년도 주행 코드)를 우리 하드웨어에 맞게 가져다 수정하는 프로젝트.

## 하드웨어 구성

| 부품 | 연결 제어기 | 방식 | 용도 |
|---|---|---|---|
| 전방 카메라 | 노트북(ROS2) | USB | 차선 인식 (YOLO 세그멘테이션) |
| 후방 카메라 | 노트북(ROS2) | USB | 후방 인식 (신규 노드 필요) |
| 후방 라이다 | 노트북(ROS2) | USB(`/dev/ttyUSB0`) | 후방 장애물 감지 → 정지 |
| 가변저항 | 아두이노 | A2핀 | 현재 조향각 피드백 |
| 조향 DC모터 | 아두이노 | PWM(2/3핀) | 목표 조향각까지 좌/우 회전 |
| 구동 DC모터 | 아두이노 | PWM(4~7핀) | 노트북이 계산한 좌/우 속도 구동 |
| 아두이노 ↔ 노트북 | — | USB 시리얼 9600bps(`/dev/ttyACM0`) | `s각도l좌속도r우속도` 문자열 |

**역할 분담**: 노트북(ROS2)이 인지·판단 전담(카메라 2대 + 라이다 → 조향각·속도 계산),
아두이노는 실행 담당(가변저항으로 조향 상태 확인하며 명령대로 모터 구동).

## 디렉토리 구조

```
SKKU_AutoDrive/
├── docs/
│   ├── ROS2_입문_가이드.md      # ★ ROS 처음이면 이것부터 (구조/실행/C↔Python)
│   ├── IMPLEMENTATION.md       # 구현 로드맵 (Phase 0~6)
│   └── code_usage_plan.md      # AutoSDV-HPC 코드 사용 계획 (파일별 수정 위치)
├── ros2_ws/src/                # ROS2 워크스페이스 (AutoSDV-HPC src/ 미러)
│   ├── camera_perception_pkg/  # 이미지 발행 + YOLO + 차선/신호등 인식
│   ├── lidar_perception_pkg/   # 라이다 수신 + 장착보정 + 장애물 판단
│   ├── decision_making_pkg/    # 경로 계산(CubicSpline) + 최종 판단
│   ├── serial_communication_pkg/ # ROS2 → 아두이노 시리얼 송신
│   └── launch_pkg/             # 통합 launch
├── arduino/
│   ├── driving/                # 조향+구동 펌웨어 (driving.ino)
│   └── check_variable_resistor/ # 가변저항 실측값 확인용
├── models/                     # YOLO 가중치 (git 제외, README에 경로만)
└── scripts/                    # 캘리브레이션/실행 헬퍼
```

## 시작하기

1. 참고 레포 클론: `git clone https://github.com/SKKUAutoLab/AutoSDV-HPC`
2. 필요한 패키지를 `ros2_ws/src/` 로 복사 후 `docs/code_usage_plan.md` 기준으로 수정
3. 빌드: `cd ros2_ws && colcon build && source install/setup.bash`
4. 실행: `ros2 launch launch_pkg main.launch.py`

## 우리 환경에서 반드시 바꿔야 하는 값 (요약)

- **시리얼 포트**: `serial_sender_node` PORT, `lidar_publisher_node` LIDAR_PORT (`ls /dev/tty*`)
- **카메라**: `image_publisher_node` `data_source:=camera`, `cam_num` (`ls /dev/video*`)
- **YOLO device**: GPU 없으면 `cuda:0` → `cpu`
- **가변저항 실측값**: `driving.ino` `resistance_most_left/right`
- **차선 원근변환 좌표**: `lane_info_extractor_node` `src_mat` (우리 카메라로 재측정)
- **신호등**: `best_urp.pt`에 `traffic_light` 클래스 없음 → HSV 직접 구현 필요
- **후진 주차**: 레포에 없음 → 신규 작성 (거리 기반 비례 정지)

자세한 파일별 위치·핵심 코드·수정 지점은 [`docs/code_usage_plan.md`](docs/code_usage_plan.md) 참고.
