# ROS2 입문 가이드 (SKKU_AutoDrive 기준)

> "ROS가 대체 뭐 하는 프로그램인지, C랑 파이썬이 어떻게 같이 도는지" 를 우리 차 코드로 설명하는 문서.
> ROS를 처음 쓰는 사람 기준으로 씀. 급하면 §1, §6, §7만 읽어도 됨.

---

## 0. 한 줄 요약

ROS2는 **하나의 프로그램이 아니라, "작은 프로그램 여러 개를 서로 대화하게 엮어주는 틀(framework)"** 이다.
우리 차는 파이썬 프로그램 여러 개(노트북)가 서로 데이터를 주고받으며 돌고, 그 결과를 **USB 케이블로 아두이노(C 프로그램)** 에 문자로 넘겨준다. ROS가 C를 실행하는 게 아니라, 파이썬 세계와 아두이노 세계가 **케이블로 편지를 주고받는** 구조다.

---

## 1. ROS2가 "무슨 프로그램"이냐?

보통 프로그램은 이렇게 생겼다:

```
main() { 시작 → 순서대로 실행 → 끝 }
```

ROS2 프로젝트는 그렇지 않다. **여러 개의 독립 프로그램(=노드)이 동시에 켜져서, 각자 계속 돌면서, 서로 데이터를 주고받는다.**

비유: 식당 주방을 생각하자.
- 재료 손질하는 사람, 굽는 사람, 접시에 담는 사람이 **각자 따로** 일한다.
- 서로 "야 이거 해줘" 하고 직접 부르지 않는다. 그냥 **패스대(통로)에 올려두면** 다음 사람이 가져간다.
- 한 명(굽는 사람)을 다른 사람으로 갈아끼워도 나머지는 그대로 돈다.

ROS2가 해주는 일은 딱 이거다:
1. 이 여러 프로그램(노드)이 **서로를 자동으로 찾게** 해주고
2. 데이터를 주고받는 **통로(토픽)** 와 **데이터 규격(메시지)** 을 표준화해주고
3. 실행/빌드/디버깅 **도구**(`ros2 run`, `ros2 topic echo` 등)를 준다.

그래서 ROS2는 "앱"이 아니라 **레고 블록을 끼우는 판 + 블록 규격**에 가깝다. 우리는 블록(노드)을 만들어 판에 꽂는다.

---

## 2. 어떻게 "동시에" 도나 — 실행 모델

- 노드 하나 = **프로세스(독립 실행 단위) 하나**. 터미널에서 `ros2 run <패키지> <노드>` 하면 그 노드 하나가 켜진다.
- 노드를 5개 켜면 프로그램 5개가 동시에 돈다. (실제로 터미널을 여러 개 열거나, `launch` 파일로 한 번에 켠다 — §7)
- 노드들은 켜지면 **네트워크로 서로를 자동 발견(discovery)** 한다. "나 `image_01` 채널에 방송하는 애야" / "나 `image_01` 듣는 애야" 하고 자기소개를 뿌리면, ROS2 하부(DDS라는 통신 계층)가 둘을 알아서 연결해준다.
- 그래서 노드를 켜는 **순서는 상관없다.** 나중에 켠 애도 알아서 낀다.

이게 일반 프로그램과 가장 큰 차이다: **중앙에서 지휘하는 main이 없다.** 각 노드가 "데이터 오면 반응"하는 방식으로 협력한다. (이런 걸 이벤트 기반 / 메시지 기반이라고 부른다.)

---

## 3. 핵심 3개념: 노드 / 토픽 / 메시지

| 개념 | 뜻 | 우리 예시 |
|---|---|---|
| **노드 Node** | 한 가지 일만 하는 작은 프로그램 (파일 하나 ≈ 노드 하나) | `yolov8_node` = YOLO 돌리는 애 |
| **토픽 Topic** | 데이터가 흐르는 **이름 붙은 통로** (라디오 채널) | `image_01`, `detections` |
| **메시지 Message** | 그 통로로 흐르는 데이터 한 덩어리 + **정해진 규격** | `Image`(영상), `MotionCommand`(조향+속도) |

노드의 두 가지 행동:
- **발행 publish** = 어떤 토픽에 데이터를 내보냄 ("이 채널에 방송")
- **구독 subscribe** = 어떤 토픽을 듣다가 데이터 오면 **콜백 함수**가 자동 실행됨

> **메시지 "규격"이란?** 채널로 아무 데이터나 못 보낸다. `MotionCommand`는 "정수 steering, 정수 left_speed, 정수 right_speed" 처럼 필드가 정해져 있다. 이 규격 정의가 `interfaces_pkg/msg/*.msg` 파일들이다. 보내는 쪽과 받는 쪽이 같은 규격을 써야 대화가 된다. (그래서 빌드할 때 이 msg들을 제일 먼저 컴파일한다.)

> **QoS 라는 말이 코드에 보임**: 채널의 "통신 품질 설정"(예: 최신 것만 받을지, 놓치면 안 되는지). 지금은 "그런 설정값이 있다" 정도만 알면 됨.

---

## 4. 우리 차의 실제 파이프라인 (진짜 토픽 이름)

```
[전방카메라]
   │ image_publisher_node        (영상 찍어서 방송)
   ▼  📻 image_01  · Image
[yolov8_node]                    (YOLO로 차선/사물 검출)
   ▼  📻 detections  · DetectionArray
[lane_info_extractor_node]       (차선 중앙점 3개 뽑기)
   ▼  📻 yolov8_lane_info  · LaneInfo
[path_planner_node]              (갈 경로 곡선 계산 = CubicSpline)
   ▼  📻 path_planning_result  · PathPlanningResult
[motion_planner_node]            (★최종 판단: 조향각+속도)
   ▲   ▲   ▲
   │   │   └─ 📻 lidar_obstacle_info · Bool   (뒤에 장애물 있나?)
   │   └───── 📻 yolov8_traffic_light_info · String (신호등 색)
   │
   ▼  📻 topic_control_signal  · MotionCommand (조향/좌속도/우속도)
[serial_sender_node]             (여기서 ROS 세계 → 아두이노 세계로 넘어감! §6)
   ▼  (USB 시리얼 케이블)
[아두이노 = C 프로그램] → 모터


라이다 가지 (독립적으로 계속 돎):
[후방라이다]
   │ lidar_publisher_node
   ▼ 📻 lidar_raw · LaserScan
[lidar_processor_node]           (장착방향 보정: offset/flip)
   ▼ 📻 lidar_processed · LaserScan
[lidar_obstacle_detector_node]   (특정 각도·거리에 물체 있나 판정)
   ▼ 📻 lidar_obstacle_info · Bool  →  위 motion_planner로 들어감
```

한 줄 흐름: **영상 → 사물인식 → 차선 → 경로 → 판단 → 시리얼 → 모터.**
화살표 하나 = 토픽(채널) 하나. 우리가 아까 고친 `motion_planner_node`가 "판단" 자리(채널 4개 듣고 1개 방송)다.

---

## 5. 노드 파일 하나 해부 — 다 똑같이 생겼다

모든 노드가 이 뼈대라, **하나만 이해하면 전부 읽힌다.**

```python
import rclpy                       # ROS2 파이썬 라이브러리 (rclpy = ROS Client Library for PYthon)
from rclpy.node import Node

class MotionPlanningNode(Node):    # 모든 노드는 Node를 상속한다
    def __init__(self):
        super().__init__('motion_planner')     # 노드 이름 등록

        # ① 시작할 때 1번: "어떤 채널을 듣고, 어떤 채널에 방송할지" 선언
        self.path_sub = self.create_subscription(
            PathPlanningResult,          # 메시지 규격
            "path_planning_result",      # 들을 채널 이름
            self.path_callback,          # 이 채널에 데이터 오면 실행할 함수
            qos_profile)
        self.publisher = self.create_publisher(
            MotionCommand, "topic_control_signal", qos_profile)  # 방송할 채널

        self.timer = self.create_timer(0.1, self.timer_callback) # 0.1초마다 실행

    # ② 구독한 채널에 메시지가 오면 ROS가 이 함수를 "자동으로" 불러줌
    def path_callback(self, msg):
        self.path_data = msg             # 일단 받아서 저장만

    # ③ 0.1초마다 실행 — 저장해둔 데이터로 판단하고 결과를 방송
    def timer_callback(self):
        ...조향/속도 계산...
        self.publisher.publish(결과_메시지)

def main():
    rclpy.init()
    node = MotionPlanningNode()
    rclpy.spin(node)                 # "계속 돌면서 메시지 기다려" — 이게 있어서 프로그램이 안 끝나고 산다
    rclpy.shutdown()
```

포인트 3가지:
1. `__init__` = 연결 선언 (한 번). `_callback` = 메시지 올 때마다 반응 (여러 번).
2. `rclpy.spin(node)` = "이벤트 기다리며 계속 돌기". 이거 때문에 노드가 안 죽고 계속 산다.
3. **당신이 예전에 바꿨던 파라미터**(`src_mat`, 각도, 속도값 등)는 이런 노드 파일 안의 **숫자 상수**다. 구조(연결)는 그대로, 숫자만 우리 하드웨어에 맞추는 게 우리 작업의 절반.

---

## 6. ⭐ C(아두이노)랑 Python(ROS)이 어떻게 같이 도나?

**이게 가장 헷갈리는 부분인데, 사실 답은 간단하다: 둘은 "같이" 안 돈다. 서로 다른 컴퓨터에서 따로 돌고, USB 케이블로 편지를 주고받을 뿐이다.**

```
┌───────────────────────────┐        USB 케이블         ┌────────────────────────┐
│  노트북 (ROS2, 파이썬)      │   (시리얼 통신, 문자열)    │  아두이노 (C 프로그램)   │
│                           │ ───────────────────────▶ │                        │
│  카메라·라이다로 판단 →     │   "s3l80r80\n"           │  문자 받아서 → 모터 구동 │
│  조향각/속도 계산           │                          │  (driving.ino)         │
│  serial_sender_node 가     │                          │  가변저항 읽어 조향 확인 │
│  이 문자열을 케이블로 보냄   │                          │                        │
└───────────────────────────┘                          └────────────────────────┘
        파이썬 세계                                            C 세계
     (ROS2가 여기서만 돎)                                 (ROS2 전혀 모름)
```

**핵심 사실들:**

1. **아두이노는 ROS 노드가 아니다.** 아두이노는 ROS를 전혀 모른다. 그냥 USB로 들어오는 글자를 읽는 별도 마이크로컨트롤러다. ROS2는 노트북 안에서만 돈다.

2. **둘을 잇는 다리 = `serial_sender_node` (파이썬).** 이 노드가 유일하게 두 세계에 걸쳐 있다:
   - 왼쪽(ROS 세계): `topic_control_signal` 채널을 **구독**해서 `MotionCommand` 메시지(조향/좌속도/우속도)를 받는다.
   - 오른쪽(C 세계): 그 값을 `"s{조향}l{좌속도}r{우속도}\n"` 같은 **문자열로 바꿔 USB 시리얼로 써 보낸다.**
   ```python
   # serial_sender_node.py 안
   ser = serial.Serial('/dev/ttyACM0', 9600)   # USB 시리얼 포트 열기
   def data_callback(self, msg):               # ROS 메시지 오면
       s = f"s{msg.steering}l{msg.left_speed}r{msg.right_speed}\n"
       ser.write(s.encode())                   # 문자열로 케이블에 씀
   ```

3. **아두이노(C)는 그 문자열을 파싱해서 모터만 돌린다.** `driving.ino`가 `s`, `l`, `r` 뒤 숫자를 읽어 조향 모터·구동 모터 PWM을 준다. 판단은 전혀 안 한다 — 시킨 대로만.

4. **왜 이렇게 나눴나?** 무거운 계산(카메라 영상처리, YOLO, 경로계산)은 힘센 노트북이, 실시간 모터 제어(PWM, 가변저항 피드백)는 아두이노가 잘한다. 각자 잘하는 걸 맡고 **케이블로 최소한의 정보(조향각/속도 3개 숫자)만** 주고받는다.

> 정리: "C랑 파이썬이 어떻게 같이 도나?"의 답 = **안 섞인다. 파이썬(ROS)이 판단해서 숫자 3개를 문자열로 만들고, USB 케이블로 아두이노(C)에 넘기면, 아두이노가 그 숫자대로 모터를 돌린다.** 언어가 섞이는 게 아니라 **케이블 하나로 편지**를 주고받는 것.

---

## 7. 실제로 어떻게 빌드하고 실행하나?

ROS2 파이썬 노드는 그냥 `python node.py`로 안 돌린다. **빌드 → 환경 등록 → 실행** 3단계다.

```bash
# (0) 매 터미널에서 ROS2 켜기 — "ROS 명령어 쓸 수 있게 환경 로드"
source /opt/ros/jazzy/setup.bash

# (1) 빌드 — 워크스페이스(ros2_ws)에서. 메시지 규격/노드를 컴파일해서 install/ 에 넣음
cd ros2_ws
colcon build
#   * colcon = ROS2용 빌드 도구 (파이썬인데 왜 빌드? → 메시지 규격 생성 + 노드를 실행 가능하게 등록하려고)

# (2) 내 워크스페이스 환경 등록 — "방금 빌드한 내 노드들을 ros2가 찾게"
source install/setup.bash

# (3) 실행 — 노드 하나씩
ros2 run camera_perception_pkg image_publisher_node
#   또는 여러 노드를 한 번에 (launch 파일, §아래)
ros2 launch launch_pkg main.launch.py
```

**launch 파일이란?** 노드 10개를 터미널 10개에서 일일이 켜기 귀찮으니, "이 노드들 이 옵션으로 한 번에 켜라"를 적어둔 파이썬 스크립트. (우리가 나중에 §7 작업에서 우리 구성용으로 새로 쓸 것.)

**디버깅 도구 (자주 씀):**
```bash
ros2 node list                     # 지금 켜져 있는 노드 목록
ros2 topic list                    # 지금 흐르는 채널 목록
ros2 topic echo /topic_control_signal   # 이 채널에 실제로 뭐가 흐르는지 실시간 출력
ros2 topic hz /image_01            # 이 채널이 초당 몇 번 방송되나
```
→ "차가 왜 안 움직이지?" 할 때 이걸로 **어느 채널에서 데이터가 끊겼는지** 추적한다. 파이프라인(§4)에서 어디까지 흐르나 보는 것.

---

## 8. 폴더 구조 용어 (워크스페이스 / 패키지)

```
ros2_ws/                     ← 워크스페이스: "프로젝트 작업 폴더" 한 개
├── src/                     ← 여기에 패키지들을 넣는다
│   ├── camera_perception_pkg/   ← 패키지: 관련된 노드들 묶음 (카메라 관련 노드 모음)
│   │   ├── package.xml           · 이 패키지가 뭘 필요로 하는지(의존성) 적은 명세
│   │   ├── setup.py              · "이 파일들을 노드로 등록" 하는 설정
│   │   └── camera_perception_pkg/
│   │       ├── image_publisher_node.py   · 노드 파일들
│   │       ├── yolov8_node.py
│   │       └── lib/                       · 노드들이 공유하는 함수 모음(우리가 복원한 func_lib!)
│   ├── lidar_perception_pkg/
│   ├── decision_making_pkg/
│   ├── serial_communication_pkg/
│   ├── interfaces_pkg/           ← 메시지 규격(.msg) 정의 패키지
│   └── launch_pkg/               ← launch 파일 모음
├── build/  install/  log/    ← colcon build가 자동 생성 (git엔 안 올림. 우리가 Phase 0에서 지운 것)
```

- **워크스페이스** = 프로젝트 폴더 (하나).
- **패키지** = 관련 노드들의 묶음 (여러 개). "카메라 패키지", "라이다 패키지"처럼 기능별로 나눔.
- **노드** = 패키지 안의 실행 프로그램 하나.

---

## 9. 용어 빠른 사전

| 용어 | 뜻 |
|---|---|
| ROS2 | 여러 프로그램(노드)을 대화하게 엮는 틀 + 도구 모음. 앱이 아니라 판. |
| 노드 Node | 한 가지 일 하는 프로그램 하나. |
| 토픽 Topic | 데이터가 흐르는 이름 붙은 채널. |
| 메시지 Message | 채널로 흐르는 데이터 + 규격. (`.msg` 파일로 정의) |
| publish/subscribe | 채널에 방송 / 채널을 들음. |
| 콜백 callback | 메시지가 오면 자동 실행되는 함수. |
| rclpy | ROS2를 파이썬에서 쓰는 라이브러리. (C++은 rclcpp) |
| DDS | 노드끼리 자동으로 찾아 연결해주는 하부 통신 계층. (신경 안 써도 됨) |
| QoS | 채널 통신 품질 설정. |
| 워크스페이스 | 프로젝트 작업 폴더 (`ros2_ws`). |
| 패키지 Package | 노드들의 묶음. |
| colcon | ROS2 빌드 도구. |
| launch 파일 | 노드 여러 개를 한 번에 켜는 스크립트. |
| `source setup.bash` | ROS/내 노드들을 터미널이 찾게 환경 로드. |

---

## 10. 그래서 우리 프로젝트에서 "내가 할 일"이 구조 어디냐

1. **파라미터 맞추기** (노드 내부 숫자): 카메라 `src_mat`, 시리얼 `PORT`, 라이다 각도, 속도값 등 → 구조는 그대로, 숫자만 우리 하드웨어로. (§5의 노드 파일들 안)
2. **새 노드/로직 추가**: 신호등 인식(§3.4), 후진 주차(§8) → 파이프라인(§4)에 채널·노드 몇 개 추가.
3. **아두이노 쪽**(`driving.ino`, C): 핀 번호·가변저항 실측값 (ROS 아님, §6의 오른쪽 세계).
4. **launch 파일**: 우리가 쓸 노드들만 한 번에 켜게 정리(§7).

전체 순서는 `docs/IMPLEMENTATION.md`(Phase 0~6) 참고.

---

## 더 궁금하면 (다음에 물어볼 것들 예시)
- "메시지 `.msg` 파일 하나 실제로 어떻게 생겼어?" → `interfaces_pkg/msg/MotionCommand.msg` 같이 보기
- "노드 하나 실제로 켜보면 어떻게 돼?" → ROS2 설치 후 `image_publisher_node`부터 하나 띄워보기
- "colcon build 하면 안에서 무슨 일이 일어나?"
- "왜 파이썬인데 빌드를 해야 해?"
