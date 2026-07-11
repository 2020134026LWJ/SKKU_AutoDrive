# 런타임 수정 이력 — Phase 1/2 통과 (2026-07-12)

`colcon build`는 처음부터 통과했다(감사 대로 하드 블로커 없음). **진짜 문제는 전부 실행할 때
드러났다.** 아래 5개를 고치고 나서야 영상 → YOLO → 차선 → 경로 → **모터 명령**까지 관통했다.

**최종 확인** (`ros2 launch launch_pkg perception.launch.py`, 하드웨어 0):
- 노드 6개 전부 생존, 죽은 노드 0
- `/topic_control_signal` 10Hz — `steering: -7, left_speed: 80, right_speed: 80`
  (아두이노로 나갈 `s-7l80r80`의 원본 값)

---

## 1. `.pyc` 로더가 복원한 `.py`를 안 씀 (3개 패키지)

**증상**: `FileNotFoundError: /home/.../Desktop/src/SKKU_AutoDrive/SKKU_AutoDrive/lib/decision_making_func_lib.cpython-310.pyc`

**원인**: 각 패키지의 `lib/__init__.py`가 원본 레포의 pyc 로더를 그대로 갖고 있었다. 그 로더는
경로를 **디렉토리 이름 조각으로 조립**하는 방식(`p[1:4] + "src" + p[5:6]*2 + "lib"`)이라
우리 워크스페이스에선 존재하지도 않는 경로를 가리킨다. Phase 0에서 `.pyc`를 디컴파일해
`.py`로 복원해 뒀는데도, `__init__.py`가 그걸 안 쓰고 여전히 pyc를 찾고 있었다.

**수정**: `lib/__init__.py` 3개(decision_making / camera_perception / lidar_perception)를
평범한 `from . import <func_lib>` 로 교체. (원본은 `__init___bak.py`로 보존)

## 2. `cv_bridge` ↔ numpy 2 충돌 → shim으로 대체

**증상**: `ImportError: A module compiled using NumPy 1.x cannot be run in NumPy 2.4.2`
→ 실제 변환 시 `KeyError: 16`. `image_publisher` / `yolov8` 노드 사망.

**원인**: ROS Jazzy의 `cv_bridge`는 numpy 1.x로 컴파일된 C 확장(`cv_bridge_boost`)에 묶여 있다.
반면 우리 pip 스택(scipy 1.18 / opencv-python 5.0 / torch 2.13+CUDA / ultralytics 8.4)은
**numpy 2를 요구**한다. numpy를 내리면 그쪽이 깨진다.

**따져본 선택지**:
| | 내용 | 왜 안 골랐나 |
|---|---|---|
| numpy 1.x로 통일 | 코드 변경 0 | scipy·opencv를 과거 버전으로 후퇴. `pip install` 한 번에 재발 |
| 가상환경 | 격리는 됨 | **numpy 제약은 그대로**(cv_bridge는 시스템 C확장) + ROS2×venv 특유의 경로 문제 |
| cv_bridge 소스 빌드 | 스택 유지 | `libopencv-dev`(sudo) 필요 + 빌드 실패 여지 |

**수정 (채택)**: `camera_perception_pkg/lib/cv_bridge_np.py` — `sensor_msgs/Image` ↔ numpy 변환을
직접 구현(약 20줄, bgr8/rgb8/mono8). 클래스·메서드 시그니처가 동일해 **쓰는 쪽은 import 한 줄만**
바뀐다. 8개 노드 적용. ROS 토픽 규격은 그대로라 rviz·rosbag 호환.

- [주의] `np.frombuffer`는 **읽기 전용** 배열을 준다 → YOLO/OpenCV의 in-place 쓰기에서
  `ValueError`가 난다. shim은 항상 복사본을 돌려준다.
- `debug_pkg`는 이 shim을 쓰므로 `package.xml`에 `<depend>camera_perception_pkg</depend>` 추가.

## 3. 테스트 영상 경로가 상대경로

**증상**: `Cannot open video file: src/camera_perception_pkg/.../driving_simulation.mp4` → 노드 사망.

**원인**: `VIDEO_FILE_PATH`가 상대경로 → **실행한 디렉토리** 기준. 원본은 레포 루트에서 실행한다고
가정했다. `ros2_ws`에서 `ros2 launch` 하면 바로 실패. 게다가 `colcon`의 install 트리엔 `.py`만
복사되고 **데이터 폴더는 안 따라간다**(그래서 `__file__` 기준 절대경로로 바꿔도 못 찾는다).

**수정**: `setup.py`가 영상을 `share/camera_perception_pkg/Collected_Datasets/`에 설치하도록 하고,
노드는 `get_package_share_directory()`로 찾는다 → 실행 위치·빌드 방식 무관.
(이미지 1259장은 무거워서 설치 안 함 — `image` 소스일 때만 쓰므로 소스 트리 경로 유지)

## 4. YOLO 모델 경로가 파일명뿐

**증상**: `Error: Model file 'best_urp.pt' not found!` → `yolov8_node`가 lifecycle activate 실패.

**원인**: 파라미터 기본값이 `"best_urp.pt"`(파일명만) → CWD 기준. 가중치는 23MB×2라 git 제외
(`.gitignore: *.pt`)이고 위치가 사람마다 다를 수 있다.

**수정**: `_resolve_model()` — 절대경로면 그대로, 아니면 후보를 순서대로 탐색.
`SKKU_MODELS_DIR` 환경변수 → 프로젝트 `models/` → AutoSDV-HPC 레포 루트 → CWD.
[주의] 런타임 `__file__`은 **install 트리**를 가리키므로 소스 기준 상대계산은 어긋난다 →
`__file__`과 CWD 양쪽에서 **위로 거슬러 올라가며** 후보를 모은다.

## 5. `motion_planner_node` 크래시 (upstream 버그)

**증상**: `AttributeError: 'NoneType' object has no attribute 'detections'` → 노드 사망.

**원인**: 디컴파일 후유증이 아니라 **원본 코드의 버그**다.
```python
elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
    for detection in self.detection_data.detections:   # ← detection_data는 None 체크 안 함
```
신호등 메시지가 첫 감지 결과보다 먼저 도착하면(시작 직후 흔한 순서) 그대로 터진다.

**수정**: `elif` 조건에 `self.detection_data is not None` 추가.

---

## 6. 프레임 80% 유실 — 커널 UDP 버퍼 < 이미지 한 장

**증상(처음 본 것)**: `/image_01`이 2~3Hz로 들쭉날쭉. `/detections` 3.9Hz.

**함정 — 측정 도구가 거짓말을 했다**: `ros2 topic hz`는 파이썬이라 **921KB 이미지 33Hz를
스스로 못 따라간다**. 이 도구가 보여준 "2Hz"는 파이프라인 속도가 아니라 **측정 도구의 한계**였다.
최소 rclpy 구독자를 직접 짜서 세어보니 퍼블리셔는 처음부터 정확히 33Hz였다.
(교훈: 큰 메시지에서 `ros2 topic hz` 숫자를 믿지 말 것)

**구간별 실측** (범인 찾기):

| 구간 | 실측 | 판정 |
|---|---|---|
| 영상 읽기+resize+변환 | 1.0 ms | 정상 |
| 발행 (퍼블리셔 자체) | **33Hz 정확** | 정상 |
| YOLO 추론 (RTX4060) | 7.6 ms (139Hz 가능) | 정상 |
| 마스크 → ROS 메시지 | 1.8 ms | 정상 |
| **BEST_EFFORT 수신** | **6.6Hz (80% 유실)** | **← 진범** |
| RELIABLE 수신 | 33.4Hz | (재전송으로 버팀) |

**원인**: 이미지 한 장 = 640×480×3 = **921KB**. 커널 UDP 버퍼 기본값(`net.core.rmem_default`)은
**208KB** — 한 장도 안 들어간다. UDP는 1.4KB 패킷 수백 개로 쪼개 보내는데 버퍼가 넘쳐 대부분
유실된다. RELIABLE은 재전송으로 버티지만, **BEST_EFFORT로 구독하는 yolov8_node는 그냥 잃는다.**

**수정 — 두 개가 다 필요하다** (하나만 하면 안 된다):

1. **커널이 허용** (sudo, 1회):
   ```bash
   echo -e 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216' \
     | sudo tee /etc/sysctl.d/60-ros2-image.conf
   sudo sysctl --system
   ```
2. **DDS가 요청** — `ros2_ws/fastdds_bigbuf.xml` (`sendSocketBufferSize`/`listenSocketBufferSize`
   = 8MB), `FASTRTPS_DEFAULT_PROFILES_FILE`로 지정. `scripts/setup_env.sh`가 자동으로 export.

> `rmem_max`는 **'허용 상한'일 뿐**이다. DDS가 명시적으로 요청하지 않으면 여전히
> `rmem_default`(208KB)를 받는다. 실제로 커널만 열었을 때는 18.4Hz까지밖에 안 올랐고,
> DDS 요청까지 넣어야 33.4Hz(유실 0)가 됐다.

**결과**: BEST_EFFORT 수신 6.6Hz → **33.4Hz (유실 0)**, `/detections` → **34Hz** (8배).

**시도했으나 효과 없던 것**: Fast DDS SHM 전용 프로파일(공유메모리). BEST_EFFORT 8.7Hz로
거의 그대로였다 → 삭제함.

---

## 실행 방법 (정리)

```bash
source ~/Desktop/Projects/SKKU_AutoDrive/scripts/setup_env.sh   # ROS + DDS 프로파일 + 모델 경로
cd ~/Desktop/Projects/SKKU_AutoDrive/ros2_ws
colcon build --symlink-install
ros2 launch launch_pkg perception.launch.py    # 하드웨어 0, 녹화 영상
```

## 남은 것 (다음 세션)

- `cv2.imshow`(SHOW_IMAGE=True)는 실측상 프레임당 비용이 크다 → 성능 필요할 땐 `logger:=false`.
- `image_publisher_node`의 `print(image_msg.header)`는 매 프레임 stdout 출력(디버그 잔재) — 제거 검토.
- Phase 3(하드웨어 브링업) 이후는 `IMPLEMENTATION.md` 참조.
