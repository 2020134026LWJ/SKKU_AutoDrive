# 실측 절차서 — 처음 하는 사람용

## 이게 뭐 하는 건가

코드에 지금 박혀 있는 숫자들은 **다른 팀 차 기준**이다. 카메라 위치도, 모터도, 바퀴도
우리 차랑 다르다. 그래서 **우리 차로 재서 그 숫자를 바꿔줘야** 한다.

하는 일은 단순하다: **자로 재고 → 숫자 하나 바꾸고 → 잘 되나 본다.** 그 반복.

---

## 시작 전에: ROS는 이 3개만 알면 된다

**새 터미널을 열 때마다 맨 처음에 이거 한 줄.** (안 하면 아래 명령이 다 "없는 명령"이라고 나온다)
```bash
cd ~/Desktop/Projects/SKKU_AutoDrive && source scripts/setup_env.sh
```

**① 프로그램 켜기**
```bash
ros2 run <패키지> <프로그램>
```
끄려면 그 터미널에서 `Ctrl+C`.

**② 숫자 바꿔서 켜기 — 실측은 사실상 이게 전부다**
```bash
ros2 run <패키지> <프로그램> --ros-args -p 숫자이름:=값
```
`--ros-args -p` 뒤에 붙이면 **코드를 안 고치고** 그 숫자만 바꿔서 실행된다.
자로 재서 나온 값을 여기 넣고 돌려보는 것의 반복이다.

**③ 프로그램이 뭘 생각하는지 엿듣기**
```bash
ros2 topic echo /<이름>
```
예: `ros2 topic echo /parking_rear_distance` → 차가 "뒷선까지 0.42m 남았다"고 **생각하는**
값이 주르륵 나온다. 그게 **줄자로 잰 실제 거리와 맞으면** 그 단계는 통과.

**끝나면 반드시**:
```bash
./scripts/stop_all.sh
```
안 끄면 프로그램이 계속 돌면서 **노트북 배터리를 갉아먹는다.** (실제로 당했다)

---

---

## 값은 전부 `config/calibration.yaml` 한 파일에 적는다

차에서 잰 숫자는 **이 파일만** 고친다. 코드는 안 건드린다.

```bash
# 노드 하나만 띄울 때 (실측 중)
ros2 run <패키지> <노드> --ros-args --params-file config/calibration.yaml

# 전체 다 띄울 때 (주행/대회) — 이 yaml을 자동으로 읽는다
ros2 launch launch_pkg main.launch.py
```

값이 제대로 먹었는지 확인:
```bash
ros2 param get /parking_controller_node speed_to_mps
```

**yaml이 아닌 것 두 가지** (여기만 코드/펌웨어를 고친다):
- **아두이노** (`arduino/driving/driving.ino`) — 가변저항 끝값, 핀 번호 → **0단계**, 한 번만
- **장치 포트/카메라 번호** — `serial_sender_node.py`, `lidar_publisher_node.py`, `main.launch.py`

---

## 도구 — 차 없이도 돌아가는 것들

| 명령 | 하는 일 |
|---|---|
| `python3 scripts/pick_points.py --cam 0` | **버드아이뷰 4점**을 화면에서 클릭해서 딴다 (전방) |
| `python3 scripts/pick_points.py --cam 1 --rear` | 후방 카메라용 + `m_per_px` 계산까지 |
| `python3 scripts/traffic_light_eval.py` | **신호등** 검출기 채점 (오검출/검출률) |
| `python3 scripts/avoid_eval.py` | **장애물 회피** 판단 채점 (8개 항목) |
| `python3 scripts/rear_park_eval.py` | **후방 카메라** 거리 정확도 채점 |
| `./scripts/test_parking.sh` | **주차 FSM** 5개 시나리오 |
| `CAMERA=1 ./scripts/test_parking.sh` | 주차 통합 (카메라→인지→판단→제어 전 체인) |
| `./scripts/stop_all.sh` | **떠 있는 노드 전부 끄기** (안 끄면 배터리가 닳는다) |

> **숫자를 헐겁게 바꿨으면 관련 채점을 다시 돌릴 것.** 특히 신호등은 임계를 풀면
> 빨간 자판기를 빨간불로 착각해서 차가 아무 데서나 선다.

---

## 준비물

- 줄자
- 마스킹 테이프 (버드아이뷰 4점용 — 바닥에 직사각형을 만든다)
- 차를 띄워둘 받침대 (바퀴가 땅에 안 닿게)
- 노트북, 차, 아두이노 케이블
- **Arduino IDE** (없으면 `sudo apt install arduino`)

**아래 순서대로 할 것.** 앞 단계가 틀리면 뒤 단계는 재봐야 헛수고다.

---

# 0단계 — 하드웨어 연결 (30분)

## 0-A. 장치를 꽂고 이름 확인

USB로 다 꽂은 뒤:

```bash
ls /dev/video*      # 카메라 (전방/후방)     예: video0, video2
ls /dev/ttyACM*     # 아두이노              예: ttyACM0   (없으면 ls /dev/ttyUSB*)
ls /dev/ttyUSB*     # 라이다                예: ttyUSB0
```

**카메라는 어느 게 전방인지 하나씩 켜보면 된다:**
```bash
ros2 run camera_perception_pkg image_publisher_node --ros-args \
  -p data_source:=camera -p cam_num:=0 -p pub_topic:=image_01 -p show_image:=true
```
숫자를 0, 1, 2… 바꿔가며 화면을 보고 정한다.

**확인한 이름을 넣을 곳:**

| 장치 | 넣을 곳 |
|---|---|
| 카메라 번호 | `main.launch.py`의 `cam_num` (전방/후방 각각) |
| 아두이노 포트 | `serial_sender_node.py:17` `PORT` |
| 라이다 포트 | `lidar_publisher_node.py:23` `LIDAR_PORT` |

## 0-B. [주의] 권한 문제 (거의 반드시 한 번 겪는다)

`Permission denied: '/dev/ttyACM0'` 이 뜨면:
```bash
sudo usermod -aG dialout $USER
```
그리고 **로그아웃했다 다시 로그인.** (재부팅하면 확실)

## 0-C. 아두이노 펌웨어 올리기 — 이걸 안 하면 아래가 전부 안 된다

**`.ino` 파일 위치** — 레포 루트의 `arduino/` 폴더:
```
arduino/
    driving/driving.ino                                    ← 최종 주행용 (이걸 올린다)
    check_variable_resistor/check_variable_resistor.ino    ← 조향 센서 끝값 재는 용
    motor_test/motor_test.ino                              ← 모터만 단독 테스트
```
Arduino IDE에서 `arduino/driving/driving.ino`를 열면 된다.
(IDE는 스케치가 같은 이름의 폴더 안에 있어야 하는데, 이미 그렇게 돼 있다)

**보드는 아두이노 메가**다. 메가는 2~13번이 전부 PWM이라 지금 핀 배치(2~7)가 그대로 동작한다.
> [주의] 우노로 바꾸면 **안 된다.** 우노는 PWM이 3,5,6,9,10,11번뿐이라 4·7번에 속도를 주면
> 모터가 그냥 꺼진다 → 전진 때 왼쪽만, 후진 때 오른쪽만 돌아서 차가 제자리에서 뱅뱅 돈다.

### ① 배선이 코드와 맞는지 확인
`driving.ino` 맨 위에 핀 번호가 있다. **실제 배선과 다르면 이 숫자를 고친다.**
```cpp
const int STEERING_1 = 2;        // 조향 모터
const int STEERING_2 = 3;
const int FORWARD_RIGHT_1 = 4;   // 오른쪽 바퀴
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 6;    // 왼쪽 바퀴
const int FORWARD_LEFT_2 = 7;
const int POT = A2;              // 조향 각도 센서(가변저항)
```

### ② 조향 센서 끝값 재기
1. Arduino IDE로 **`check_variable_resistor.ino`를 업로드**
2. **시리얼 모니터**를 연다 (IDE 오른쪽 위 돋보기 아이콘, 9600 baud)
3. 핸들을 **손으로 왼쪽 끝까지** 돌렸을 때 숫자를 읽는다 → 예: 455
4. **오른쪽 끝까지** 돌렸을 때 숫자를 읽는다 → 예: 348
5. 그 두 값을 `driving.ino`에 넣는다:
   ```cpp
   const int resistance_most_left  = 455;   // ← 왼쪽 끝에서 읽은 값
   const int resistance_most_right = 348;   // ← 오른쪽 끝에서 읽은 값
   ```

### ③ `driving.ino` 업로드
이게 최종 주행 펌웨어다. **한 번 올리면 끝** — 이후로는 노트북(yaml)만 고치면 된다.

### ④ (선택) 모터만 먼저 확인
`motor_test.ino`를 올리면 바퀴가 도는지 단독으로 볼 수 있다.

> 노트북과 아두이노는 **9600 baud 시리얼**로 `s0l80r80` 같은 문자열을 주고받는다.
> (s=조향, l=왼쪽, r=오른쪽) 이 규약은 이미 맞춰져 있으니 건드릴 것 없다.

---

# 1단계 — 바퀴가 어느 쪽으로 꺾나 (5분)

### 왜 제일 먼저?
이게 반대면 차가 주차칸 **반대쪽**으로 꺾는다. 모르고 아래를 다 재면 **전부 버려야 한다.**
그리고 이건 컴퓨터로는 확인할 방법이 없다. 차를 띄워놓고 한 번 쏴보는 수밖에 없다.

### 하는 법
차를 받침대에 올려 **바퀴가 땅에 안 닿게** 한다. 터미널 3개를 연다.

터미널 1:
```bash
ros2 run decision_making_pkg parking_controller_node
```
터미널 2:
```bash
ros2 run decision_making_pkg motion_planner_node
```
터미널 3 — 주차 시작 신호를 쏜다:
```bash
ros2 topic pub -1 /parking_trigger std_msgs/Bool "{data: true}"
```

### 뭘 보나
**앞바퀴가 주차칸 쪽으로 꺾이면 통과.**

**반대로 꺾이면** — 코드 안 고쳐도 된다. 터미널 1을 Ctrl+C로 끄고 이렇게 다시 켠다:
```bash
ros2 run decision_making_pkg parking_controller_node --ros-args -p slot_side:=left
```
그리고 이 `left`를 **기록표에 적어둔다.** 앞으로 주차를 켤 땐 항상 이걸 붙여야 한다.

---

# 2단계 — 모터가 실제로 도는 최소 세기 (10분)

### 왜?
모터 세기(PWM)를 너무 낮게 주면 차가 "천천히" 가는 게 아니라 **아예 안 움직인다.**
주차 마지막에 살살 다가갈 때 이 값 아래로 내려가면 차가 멈춘 채 시간만 흐른다.

### 하는 법
차를 바닥에 내려놓는다. 모터 세기를 **20 → 30 → 40 → 50** 순으로 올리며
**차가 처음 구르기 시작하는 값**을 찾는다.

```bash
# 세기 30으로 테스트 (양쪽 바퀴 30, 조향 0)
ros2 topic pub -1 /topic_control_signal interfaces_pkg/MotionCommand \
  "{steering: 0, left_speed: 30, right_speed: 30}"
```
숫자만 바꿔가며 반복. **안 움직이면 다음 숫자로.**

### 기록
차가 구르기 시작한 값을 적는다 (예: 45). → 기록표 `min_move_speed`

---

# 3단계 — 세기 vs 실제 속도 (15분) 중요

### 왜?
우리 차엔 **바퀴가 몇 바퀴 돌았는지 세는 센서가 없다.** 그래서 "얼마나 갔나"를
오직 이 값으로 추측한다. 주차할 때 뒷선이 안 보이는 마지막 구간의 안전이
**전부 이 값에 걸려 있다.** 틀리면 덜 들어가거나 **주차칸을 뚫고 나간다.**

### 하는 법
1. 출발선에 테이프를 붙인다
2. 뒤로 **정확히 3초** 굴린다 (세기 70)
   ```bash
   ros2 topic pub -1 /topic_control_signal interfaces_pkg/MotionCommand \
     "{steering: 0, left_speed: -70, right_speed: -70}"
   ```
   3초 뒤 정지:
   ```bash
   ros2 topic pub -1 /topic_control_signal interfaces_pkg/MotionCommand \
     "{steering: 0, left_speed: 0, right_speed: 0}"
   ```
3. **간 거리를 줄자로 잰다** (예: 63 cm = 0.63 m)
4. 계산: `0.63 ÷ (70 × 3) = 0.003`

### 기록
나온 숫자 → 기록표 `speed_to_mps`

**3번 재서 비슷하게 나오는지 확인할 것.** 많이 다르면 바닥이 미끄러운 것 — 대회장 바닥에서 재라.

---

# 4단계 — 앞 카메라: 버드아이뷰 4점 (30분)

### 버드아이뷰가 뭔가
카메라로 도로를 찍으면 **비스듬히** 보인다. 실제로는 평행한 차선이 화면에선 위로 갈수록
좁아진다(사다리꼴). 이 상태로는 곡선이 얼마나 휘었는지 제대로 못 잰다.

그래서 **위에서 내려다본 것처럼 펴야** 하는데, 컴퓨터는 어디까지가 바닥인지 모른다.
→ **"화면의 이 네 점이 실제로는 직사각형이다"** 를 사람이 알려줘야 한다. 그게 4점이다.

```
카메라 화면                          펴진 화면 (버드아이뷰)
┌────────────────┐                  ┌────────────────┐
│      ①────②    │                  │  ①────────②    │
│     ╱      ╲   │   ─────────▶     │  │        │    │
│    ╱        ╲  │                  │  │        │    │
│   ④──────────③ │                  │  ④────────③    │
└────────────────┘                  └────────────────┘
   사다리꼴로 보임                     직사각형이 된다
```

### 하는 법 — 클릭 도구를 쓴다 (숫자를 손으로 타이핑하지 않는다)

1. 바닥에 **마스킹 테이프로 직사각형**을 만든다
2. ```bash
   python3 scripts/pick_points.py --cam 0
   ```
3. 창이 뜨면 네모의 꼭짓점을 **좌상 → 우상 → 우하 → 좌하** 순서로 클릭
4. 4개를 찍으면 **펴진 화면이 같이 뜬다** → 네모가 **진짜 직사각형**이면 성공
5. `s`를 누르면 yaml에 붙여넣을 줄이 터미널에 나온다 (`r`=다시 찍기)

### [주의] 네모를 크게 만들 것
**네모 바깥은 펴진 화면에서 잘려나간다.** 작게 만들면 앞쪽 차선이 잘려서 경로를 못 그린다.
**보고 싶은 바닥 범위 전체**를 감싸게 잡을 것.

### 확인
```bash
ros2 run camera_perception_pkg lane_info_extractor_node --ros-args \
  -p show_image:=true --params-file config/calibration.yaml
```
`lane2_bird_img` 창에서 **차선이 세로로 곧게** 보이면 맞은 것.

> 창은 **캘리브레이션할 때만** 켠다. 평소엔 꺼둔다 (매 프레임 비용 + 이미지 토픽이
> DDS를 한 벌 더 흘려서 프레임 유실을 유발한다).

### 같이 잡을 것
- **앞범퍼 중심이 화면의 몇 픽셀인가** → `car_center_point` (기본 320,179)
- **`lane_width`** (기본 300) — [주의] **cm 아니라 픽셀이다.** 한쪽 차선만 보일 때
  "반대쪽은 이만큼 떨어져 있겠거니" 하고 중앙을 추정하는 값

### 기록
`config/calibration.yaml` 의 `lane_info_extractor_node` / `path_planner_node` 절.
**코드는 안 건드린다.**

---

# 5단계 — 핸들과 속도 (20분)

트랙을 실제로 달려보며 조정한다.

- **코너에서 안쪽으로 파고들면** → 핸들을 덜 꺾게 (`motion_planner_node.py`의 `52`를 키운다)
- **코너에서 밖으로 나가면** → 반대로 줄인다
- **한쪽으로 쏠리며 가면** → 좌우 바퀴 속도를 다르게 준다 (기본 둘 다 80)

### 통과 기준
트랙 한 바퀴를 차선 안에서 돈다.

---

# 6단계 — 뒤 카메라 (주차의 눈) (30분) 중요

### 왜?
우리 주차장은 **뒷벽이 없다.** 양옆에 차만 있다. 그래서 차가 멈출 근거는
**바닥에 그려진 주차칸 뒤 경계선**밖에 없다. 그 선까지 몇 m 남았는지 재는 게 이 단계다.

### 하는 법 — 테이프 네모 하나로 두 값을 한 번에 잡는다

1. 차 뒤 바닥에 **마스킹 테이프로 직사각형**을 만든다.
   [주의] **크게 만들 것** — 범퍼 바로 뒤부터 **1m 이상**까지 덮게.
   네모 바깥은 잘려나가서, 작으면 **주차칸 뒤 경계선이 아예 안 보이고 차가 안 멈춘다.**
2. 그 **세로 길이를 줄자로 재둔다** (예: 1.2 m)
3. ```bash
   python3 scripts/pick_points.py --cam 1 --rear
   ```
4. 네모의 꼭짓점을 **좌상 → 우상 → 우하 → 좌하** 순서로 클릭
5. 펴진 화면에서 **네모가 직사각형**이고 **주차칸 뒤 경계선이 보이면** 성공
6. `s`를 누르면 `src_mat` 줄과 **`m_per_px` 계산법**이 같이 나온다
   → `m_per_px = 실제 세로 길이 ÷ 480`  (예: `1.2 ÷ 480 = 0.0025`)

### 통과 기준
값을 yaml에 넣고 노드를 켠 뒤:
```bash
ros2 run camera_perception_pkg rear_park_detector_node \
  --ros-args --params-file config/calibration.yaml
ros2 topic echo /parking_rear_distance      # 다른 터미널
```
차를 손으로 밀어 뒤로 보내면서, **나오는 숫자가 줄자로 잰 실제 거리와 3cm 안쪽으로**
맞으면 통과.

### 기록
`src_mat` / `m_per_px` / `cam_offset_m`(렌즈에서 뒤범퍼 끝까지 거리)
→ `config/calibration.yaml` 의 `rear_park_detector_node` 절

---

# 7단계 — 뒷선이 안 보이기 시작하는 거리 (10분) 중요

### 왜?
차가 주차칸에 다 들어가면 뒷선이 **범퍼 밑으로 사라진다.** 카메라가 못 본다.
그게 사실은 **"다 들어왔다"는 신호**다. 이 거리를 알아야 차가 그걸 고장으로
착각하지 않는다.

### 하는 법
6단계처럼 거리를 띄워놓고:
```bash
ros2 topic echo /parking_rear_distance
```
차를 **아주 천천히** 뒤로 민다. 어느 순간 **숫자가 안 나오기 시작한다.**
그 순간 멈추고, **뒷선까지 실제 거리를 줄자로 잰다** (예: 22 cm).

### 기록
잰 값보다 **조금 크게** 적는다. 22cm가 나왔으면 **0.30**으로.

> 왜 크게? 숫자가 끊기기 직전의 마지막 값은 항상 그 문턱보다 살짝 바깥이다.
> 딱 맞춰놓으면 정상 주차를 고장으로 오해해서 멈춰버린다.

→ 기록표 `blind_enter_dist`

---

# 8단계 — 주차 (40분, 제일 오래 걸림)

앞 단계를 다 맞춘 뒤에 한다. 지금까지 잰 값들을 전부 넣고 켠다:

```bash
ros2 run decision_making_pkg parking_controller_node --ros-args \
  -p slot_side:=right \
  -p min_move_speed:=45 \
  -p speed_to_mps:=0.003 \
  -p blind_enter_dist:=0.30
```
(값은 기록표에서 가져온다)

### 순서대로 조정
1. **정렬 전진** (`align_duration`, 기본 1.0초) — 주차칸을 얼마나 지나쳐서 멈출지
2. **후진 속도** (`reverse_speed`, 기본 70)
3. **핸들 꺾고 후진하는 시간** (`turn_duration`, 기본 2.5초)
   → **주차 정확도를 좌우하는 값.** 차가 90도 도는 데 걸리는 시간이다.
   **다른 값을 다 맞춘 뒤 마지막에 0.5초씩 바꿔가며** 잡는다.

### 통과 기준
**3번 연속으로 칸 안에 들어가고, 뒷선을 넘지 않는다.**

---

# 9단계 — 장애물 회피 (차선 변경) (30분) 미션 1

### 뭘 하는 건가
차선을 달리다가 앞에 **차 모양 장애물**이 나타나면 **옆 차선으로 피했다가 돌아온다.**
2~3번 연속으로 나오고, 위치는 랜덤이다.

핸들을 억지로 꺾는 게 아니라 **"따라갈 차선"을 바꾼다.** 그러면 기존 차선 추종이
알아서 부드럽게 옮겨 간다.

### 가장 먼저: lane1 / lane2 중 어느 게 왼쪽인가
YOLO 모델이 차선을 `lane1`, `lane2` 두 개로 구분해서 학습돼 있는데,
**어느 게 왼쪽/오른쪽인지는 코드로 알 수 없다** (모델 만들 때 정해진 것).
눈으로 확인해야 한다.

```bash
ros2 run camera_perception_pkg lane_info_extractor_node --ros-args \
  -p show_image:=true -p target_lane:=lane2
```
창을 보면서 **차가 지금 달리는 차선이 lane2인지** 확인한다.
아니면 `-p target_lane:=lane1`로 바꿔본다.

- 지금 달리는 차선 → `home_lane`
- 피할 때 갈 차선 → `other_lane`

### 그다음 3개를 잡는다
차 앞에 장애물을 놓고 카메라 화면을 보면서:

- **`trigger_y`** (기본 300) — 장애물이 **얼마나 가까워지면** 피할지.
  화면에서 장애물 상자의 **아래변이 몇 픽셀**일 때 피하기 시작할지.
  값이 크면 더 가까이 붙어서 피한다 (늦게 피함).
- **`lane_half_width`** (기본 120) — 장애물이 **내 차선 안에 있나** 판정.
  화면 중앙에서 좌우 몇 픽셀까지를 "내 차선"으로 볼지.
  **옆 차선에 있는 차를 자꾸 피하려 들면 이 값을 줄인다.**
- **`pass_clearance_m`** (기본 0.40 m) — 피한 뒤 **얼마나 더 가야 돌아올지.**
  [주의] 장애물이 화면에서 사라졌다고 바로 돌아오면 **옆구리를 긁는다.**
  차 길이 + 여유로 잡을 것.

### 실행
```bash
ros2 run decision_making_pkg obstacle_avoider_node --ros-args \
  -p home_lane:=lane2 -p other_lane:=lane1 -p trigger_y:=300
```

### 통과 기준
```bash
ros2 topic echo /avoid_state      # FOLLOW / AVOID / RETURN 이 보인다
ros2 topic echo /target_lane      # lane2 -> lane1 -> lane2 로 바뀐다
```
- 장애물 앞에서 **옆 차선으로 옮긴다**
- **완전히 지나간 뒤** 원래 차선으로 돌아온다 (긁지 않는다)
- 옆 차선에 서 있는 차를 보고 **또 피하지 않는다**
- 연속 2~3개를 각각 다 피한다

바꾼 숫자로 다시 채점하려면:
```bash
python3 scripts/avoid_eval.py     # 8개 항목 전부 통과해야 한다
```

---

# 10단계 — 신호등 (20분)

### 재야 할 건 사실상 하나
신호등 **램프(동그란 불)가 화면에서 몇 픽셀 크기로 보이나.**

정지선 앞에 차를 세우고 카메라 화면을 본다. 램프 지름이 대략 몇 픽셀인지 보고,
그 넓이가 `min_area`(60) ~ `max_area`(8000) 사이에 들어오는지 확인한다.

### 통과 기준
- 빨간불 앞에서 **선다**
- 초록불에서 **간다**
- **신호등이 없는 구간에서 절대 서지 않는다** ← 이게 제일 중요

### [주의] 경고
숫자를 헐겁게(`max_area`를 키우거나 `val_min`을 낮추면) 바꾸면 **빨간 자판기나
주황색 기둥을 빨간불로 착각한다.** 그럼 차가 아무 데서나 선다.

그래서 이 숫자를 바꿨으면 **반드시** 이걸 돌려서 확인:
```bash
python3 scripts/traffic_light_eval.py
```
마지막 줄에 **"오검출 0"**이 나와야 한다.

---

# 11단계 — 라이다 (20분)

라이다는 뒤쪽에 달린다. 라이다가 생각하는 "0도"와 우리가 원하는 "정면"이 다르므로
그 각도차를 맞춰줘야 한다 (`lidar_processor_node.py`의 `offset`).

### 통과 기준
```bash
ros2 topic echo /lidar_obstacle_info
```
켜놓고 **차 앞에 손을 대면 `true`가 뜬다.**

---

> **아두이노는 0단계에서 이미 끝났다.** (펌웨어를 안 올리면 1단계부터 아무것도 안 움직인다)

---

# 실측값 기록표 — 여기 채워넣을 것

> 값은 `config/calibration.yaml` 에 적는다. 이 표는 **한눈에 보는 요약**이다.
>
> **한 번에 하나씩만 바꿀 것.** 두 개를 동시에 바꾸면 뭐가 효과를 냈는지 알 수 없다.
> 잘 되는 값이 나오면 **바로 yaml에 적어둘 것.** "아까 그 값이 뭐였지"가 반드시 생긴다.

| 항목 | 이름 | 기본값 | **우리 차 값** | 잰 날 |
|---|---|---|---|---|
| 바퀴 꺾이는 방향 | `slot_side` | right | | |
| 모터 최소 세기 | `min_move_speed` | 45 | | |
| 세기→속도 | `speed_to_mps` | 0.003 | | |
| 뒤 카메라 네 점 | `src_mat` | (임시값) | | |
| 픽셀→미터 | `m_per_px` | 0.0022 | | |
| 렌즈~범퍼 거리 | `cam_offset_m` | 0.10 | | |
| 뒷선 사라지는 거리 | `blind_enter_dist` | 0.30 | | |
| 핸들 꺾고 후진 시간 | `turn_duration` | 2.5 | | |
| 달리는 차선 | `home_lane` | lane2 | | |
| 피할 차선 | `other_lane` | lane1 | | |
| 회피 시작 거리 | `trigger_y` | 300 | | |
| 내 차선 폭 | `lane_half_width` | 120 | | |
| 복귀 전 전진거리 | `pass_clearance_m` | 0.40 | | |
| 신호등 램프 크기 | `min/max_area` | 60/8000 | | |
| 라이다 각도 | `offset` | 0 | | |
| 가변저항 좌/우 | (아두이노) | 460/352 | | |

**주차를 켤 땐 이 값들을 전부 붙여서 켠다:**
```bash
ros2 run decision_making_pkg parking_controller_node --ros-args \
  -p slot_side:=<값> -p min_move_speed:=<값> -p speed_to_mps:=<값> \
  -p blind_enter_dist:=<값> -p turn_duration:=<값>
```
