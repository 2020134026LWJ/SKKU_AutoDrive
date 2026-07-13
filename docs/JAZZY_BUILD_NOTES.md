# Jazzy 빌드 정밀 감사 결과 (2026-07-11)

Humble(py3.10) → Jazzy(py3.12) + 최신 라이브러리(numpy 2.x 등) 이식에서 **colcon build / 실행 시 깨질 지점**을 정적으로 점검한 결과.
> [주의] ️ 이건 정적 감사라 위험을 줄이는 것. 진짜 확인은 ROS2 설치 후 실제 `colcon build`.

## 결론: 하드 블로커 없음 ✅

우리 6개 패키지(camera/lidar/decision/serial/interfaces/launch)에서 **빌드를 막는 확정 문제는 발견 안 됨.**

| 점검 항목 | 결과 |
|---|---|
| numpy 2.x 제거 별칭 (`np.float`/`np.int`/`np.NaN` 등) | ✅ 사용 안 함 |
| `distutils`/`imp` (py3.12에서 제거) | ✅ 사용 안 함 |
| `collections.Mapping` 등 (py3.10+ 이동) | ✅ 사용 안 함 |
| package.xml 의존성 (cv_bridge/message_filters/tf2/메시지들) | ✅ 전부 Jazzy apt에 존재 |
| `message_filters` 사용 | ✅ 없음 (신호등 노드 재작성 때 제거됨) |
| rclpy 옛 API (create_rate 등) | ✅ 특이 없음 |
| ultralytics 내부 import (`ultralytics.engine.results`) | ✅ 8.4.92에서 정상 |
| 노드 실행 등록 (entry_points) | ✅ 전부 등록됨 |

## 지켜볼 것 1개 (블로커 아님, 경고 수준)

**setup.cfg의 옛 옵션** — 4개 파이썬 패키지에 아래가 있음:
```ini
[develop]
script_dir=$base/lib/<pkg>
[install]
install_scripts=$base/lib/<pkg>
```
- 이건 ROS2 옛 템플릿 잔재. 신형 setuptools에서 **deprecation 경고**를 냄.
- 현재 환경의 setuptools는 **83.0.0**(`~/.local`, torch/ultralytics 설치 때 딸려옴). 확인 결과 **`develop` 명령은 아직 존재** → 빌드는 됨(경고만).
- **만약** 나중에 `colcon build`가 setuptools 관련 에러로 실패하면, 그때만 아래로 내리면 됨(약 1MB):
  ```bash
  pip install --user --break-system-packages "setuptools<80"
  ```
  (선제적으로 지금 할 필요는 없음. 빌드가 실제로 깨질 때만.)

## 빌드 순서 (WiFi 후)

```bash
# 1. ROS2 설치 (scripts/install_ros2_jazzy.sh)
# 2. 빌드
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/Projects/SKKU_AutoDrive/ros2_ws
colcon build            # interfaces_pkg(메시지) 먼저 빌드됨
# 3. 환경 등록 + 실행
source install/setup.bash
ros2 launch launch_pkg perception.launch.py   # 하드웨어 없이 먼저 테스트
```

빌드가 에러 나면: 에러 메시지에 `setuptools`/`develop` 있으면 위 §setuptools 다운그레이드, 그 외엔 메시지 그대로 가져와서 확인.
