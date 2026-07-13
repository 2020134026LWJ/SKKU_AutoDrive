# 레포 코드 사용 계획 (구체적 파일/코드 기준)

레포: SKKUAutoLab/AutoSDV-HPC
아래는 우리 하드웨어(아커만 섀시, 전/후 카메라, 후방 라이다, 가변저항+DC모터 조향, 초음파 미사용)에 실제로 가져다 쓸 코드의 정확한 위치, 핵심 코드, 수정할 부분을 정리한 것.

---

## 0. 요약: 하드웨어 ↔ 제어기 연결

| 부품 | 연결 제어기 | 연결 방식 | 용도 |
|---|---|---|---|
| 전방 카메라 | 노트북(ROS2) | USB | 차선(`lane1`/`lane2`) 인식용 YOLO 세그멘테이션 입력, 이후 경로계산에 사용 |
| 후방 카메라 | 노트북(ROS2) | USB | 현재 레포엔 활용 코드 없음. 신규 노드 작성 필요 (모니터링/후방 인식 등) |
| 후방 라이다 | 노트북(ROS2) | USB(시리얼, `/dev/ttyUSB0`) | 후방 장애물 감지 → 감지되면 정지 명령 |
| 가변저항 | 아두이노 | 아날로그 입력(A2핀) | 현재 조향각 피드백 — 목표각과 비교해서 조향 모터를 어느 방향으로 돌릴지 결정 |
| 조향 DC모터 | 아두이노 | 디지털 출력(PWM, 2/3번 핀) | 가변저항 값이 목표 조향각과 같아질 때까지 좌/우로 회전 |
| 좌/우 구동 DC모터 | 아두이노 | 디지털 출력(PWM, 4~7번 핀) | 노트북이 계산한 좌/우 속도값을 그대로 받아 구동 (피드백 없음) |
| 아두이노 ↔ 노트북 | — | USB 시리얼(9600bps, `/dev/ttyACM0`) | 노트북(ROS2)이 카메라·라이다로 계산한 조향각/속도를 `s각도l좌속도r우속도` 문자열로 아두이노에 전달 → 아두이노가 이 값대로 모터만 구동 |

즉 **노트북이 인지·판단을 전담(카메라 2대+라이다 데이터를 받아 조향각·속도를 계산)**하고, **아두이노는 가변저항으로 조향 상태만 확인하면서 노트북이 내려준 명령대로 모터를 움직이는 실행 담당**인 구조.

레포의 라이다 로직은 "감지되면 정지"라는 회피용 판단만 있어서, 후진 주차처럼 장애물에 가까이 다가가야 하는 동작에는 별도 로직이 필요함 (8번 항목 참고).

---

## 1. 조향 + 구동 (아두이노)

**경로**: `arduino/driving/driving.ino`

```cpp
const int POT = A2;
const int resistance_most_left = 460;
const int resistance_most_right = 352;
const int MAX_STEERING_STEP = 7;
const int STEERING_SPEED = 128;

resistance = analogRead(POT);
mapped_resistance = map(resistance, resistance_most_left, resistance_most_right,
                         -MAX_STEERING_STEP, MAX_STEERING_STEP + 1);

if (mapped_resistance == angle) maintainSteering();
else if (mapped_resistance > angle) steerLeft();
else steerRight();
```

시리얼로 `s<angle>l<left>r<right>\n` 문자열을 받으면 `angle`은 -7~7 범위로 클램프되고(`processData` 함수), 조향 모터는 목표각과 다르면 무조건 고정 속도(`STEERING_SPEED=128`)로 움직이고 같아지면 멈추는 방식(비례 제어 아님).

**수정할 부분**
- `resistance_most_left`, `resistance_most_right` — 이건 원 팀 가변저항 실측값. `arduino/check_variable_resistor/check_variable_resistor.ino`(A2핀 아날로그값을 시리얼로 출력하는 코드)를 그대로 업로드해서, 우리 가변저항을 좌/우 끝까지 돌렸을 때 나오는 값 2개로 교체
- `STEERING_1`(2), `STEERING_2`(3), `FORWARD_RIGHT_1/2`(4,5), `FORWARD_LEFT_1/2`(6,7) 핀 번호 — 실제 모터 드라이버 배선에 맞게 수정
- (선택) 조향을 비례 제어로 바꾸려면 `steerLeft()`/`steerRight()`에서 쓰는 고정값 `STEERING_SPEED` 대신, `abs(mapped_resistance - angle)`에 비례하는 speed 값을 `analogWrite`에 넘기도록 수정 — 목표각 근처에서 자연스럽게 감속됨

---

## 2. 시리얼 통신 (ROS2 → 아두이노)

**경로**: `src/serial_communication_pkg/serial_communication_pkg/serial_sender_node.py`

```python
SUB_TOPIC_NAME = "topic_control_signal"
PORT = '/dev/ttyACM0'
ser = serial.Serial(PORT, 9600, timeout=1)

def data_callback(self, msg):
    serial_msg = PCFL.convert_serial_message(msg.steering, msg.left_speed, msg.right_speed)
    ser.write(serial_msg.encode())
```

메시지 포맷은 `src/serial_communication_pkg/serial_communication_pkg/lib/protocol_convert_func_lib.py`에서 결정:
```python
def convert_serial_message(steering, left_speed, right_speed):
    return f"s{steering}l{left_speed}r{right_speed}\n"
```

**수정할 부분**
- `PORT` — 아두이노 연결 후 `ls /dev/ttyACM*` 또는 `ls /dev/ttyUSB*`로 실제 장치명 확인해서 교체 (아두이노 보드 종류에 따라 ACM이 아니라 USB로 잡힐 수 있음)
- 이 파일 자체는 로직 수정 불필요, 포트 이름만 맞으면 그대로 사용 가능

---

## 3. 전방 카메라 파이프라인

### 3.1 이미지 발행

**경로**: `src/camera_perception_pkg/camera_perception_pkg/image_publisher_node.py`

```python
PUB_TOPIC_NAME = 'image_01'
DATA_SOURCE = 'video'   # 기본값이 카메라가 아니라 저장된 영상 파일임
CAM_NUM = 0

if self.data_source == 'camera':
    self.cap = cv2.VideoCapture(self.cam_num)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

**수정할 부분**
- `DATA_SOURCE` 기본값이 `'video'`(레포 안 mp4 샘플 재생)로 되어 있음. 실제 웹캠 쓰려면 노드 실행 시 파라미터로 `data_source:='camera'`, `cam_num:=0`(전방 카메라 장치번호, `ls /dev/video*`로 확인) 넘겨야 함
- 후방 카메라는 이 노드를 하나 더 띄워야 하는데, `PUB_TOPIC_NAME='image_01'`이 코드에 고정값으로 박혀있고 `pub_topic`은 파라미터로는 열려있음 — 실행 시 `pub_topic:='image_02'`, `cam_num:=1`로 별도 인스턴스 실행하면 됨 (노드 이름도 겹치지 않게 `name:=` 지정 필요)

### 3.2 YOLO 세그멘테이션 (차선/신호등 인식용 딥러닝 모델)

**경로**: `src/camera_perception_pkg/camera_perception_pkg/yolov8_node.py`, 모델 파일: `best_urp.pt`(레포 루트)

```python
self.declare_parameter("model", "best_urp.pt")
self.declare_parameter("device", "cuda:0")
...
self._sub = self.create_subscription(Image, "image_01", self.image_cb, ...)  # 토픽명이 코드에 고정돼 있음
```

`best_urp.pt`를 직접 열어서 확인한 결과, 학습된 클래스는 **`car`, `center_line`, `lane1`, `lane2`, `left_line`, `right_line`** 6개뿐이고 **`traffic_light` 클래스는 없음**.

**수정할 부분**
- `device` — GPU(cuda:0) 없으면 `"cpu"`로 변경해야 실행됨. 노트북에 엔비디아 GPU 없으면 필수 수정 사항
- **신호등 인식이 이 모델로는 애초에 동작 안 함** — 아래 3.4 신호등 항목에서 대안 설명
- 후방 카메라에 YOLO를 추가로 돌리려면, 이미지 구독 토픽("image_01")이 코드에 하드코딩돼 있어서 파라미터로는 못 바꿈. 이 파일을 복사해서 두 번째 노드(예: `yolov8_node_rear.py`)로 만들고 구독 토픽만 "image_02"로 고쳐야 함

### 3.3 차선 인식

**경로**: `src/camera_perception_pkg/camera_perception_pkg/lane_info_extractor_node.py`

```python
lane2_edge_image = CPFL.draw_edges(detection_msg, cls_name='lane2', color=255)

dst_mat = [[round(w*0.3), 0], [round(w*0.7), 0], [round(w*0.7), h], [round(w*0.3), h]]
src_mat = [[238, 316], [402, 313], [501, 476], [155, 476]]
lane2_bird_image = CPFL.bird_convert(lane2_edge_image, srcmat=src_mat, dstmat=dst_mat)
roi_image = CPFL.roi_rectangle_below(lane2_bird_image, cutting_idx=250)

grad = CPFL.dominant_gradient(roi_image, theta_limit=70)
for target_point_y in range(5, 155, 50):
    target_point_x = CPFL.get_lane_center(roi_image, detection_height=target_point_y,
                                           detection_thickness=10, road_gradient=grad, lane_width=300)
```

이 노드는 YOLO가 찾아낸 `lane2` 클래스의 세그멘테이션 결과를 입력받아, 원근변환(bird's eye view)으로 위에서 내려다본 형태로 바꾼 다음 차선 중앙 지점 3개를 뽑아냄.

**수정할 부분**
- `src_mat`의 4개 좌표(238,316 등)는 원 팀 카메라 해상도·장착 각도 기준으로 잡은 값이라 100% 다시 잡아야 함 — 우리 카메라로 찍은 실제 프레임(640x480)에 차선 사각 영역 4꼭짓점을 직접 찍어서 좌표 교체
- `lane_width=300`, `detection_thickness=10`, `cutting_idx=250` — 트랙 폭·카메라 높이에 따라 실측 후 조정
- `CPFL.draw_edges`, `CPFL.bird_convert`, `CPFL.get_lane_center` 등의 실제 내부 구현은 `src/camera_perception_pkg/camera_perception_pkg/lib/camera_perception_func_lib.py`에 있어야 하는데, **레포에 소스(.py)가 없고 컴파일된 `.pyc`만 들어있음.** 실행 자체는 이 `.pyc`로 문제없이 되지만, 내부 알고리즘을 직접 뜯어고쳐야 하는 상황이 오면(예: 원근변환 방식 자체를 바꾸고 싶을 때) 이 함수들을 새로 작성해야 함

### 3.4 신호등 인식

**경로**: `src/camera_perception_pkg/camera_perception_pkg/traffic_light_detector_node.py`

```python
for detection in detection_msg.detections:
    if detection.class_name == 'traffic_light':
        hsv_ranges = {
            'red1': (np.array([0,100,95]), np.array([10,255,255])),
            'red2': (np.array([160,100,95]), np.array([179,255,255])),
            'yellow': (np.array([20,100,95]), np.array([30,255,255])),
            'green': (np.array([40,100,95]), np.array([90,255,255]))
        }
        traffic_light_color = CPFL.get_traffic_light_color(cv_image, detection.bbox, hsv_ranges)
```

이 코드는 YOLO가 `traffic_light` 클래스로 박스를 잡아준 다음, 그 박스 안에서만 HSV 색상 범위로 빨강/노랑/초록을 구분하는 방식. **그런데 `best_urp.pt`에 `traffic_light` 클래스가 없어서, `detection.class_name == 'traffic_light'` 조건이 절대 참이 안 되고 이 코드는 항상 `'None'`만 출력함.**

**수정 방향 (택1)**
- (A) YOLO 없이 직접 구현: 신호등이 있을 법한 화면 영역(위쪽 절반 등)에 대해 `cv2.inRange`로 위 HSV 범위를 전체 프레임에 바로 적용해서 빨간/초록 픽셀 덩어리(contour)를 찾는 방식으로 이 노드를 다시 작성. `get_traffic_light_color` 함수 로직(HSV 범위)은 그대로 재사용 가능, `detection.bbox` 의존만 제거하면 됨
- (B) `best_urp.pt`를 신호등 이미지까지 포함해서 재학습 — 데이터 라벨링과 학습 과정이 필요해서 시간이 많이 듦

---

## 4. 후방 라이다

### 4.1 데이터 수신

**경로**: `src/lidar_perception_pkg/lidar_perception_pkg/lidar_publisher_node.py`

```python
PUB_TOPIC_NAME = 'lidar_raw'
LIDAR_PORT = '/dev/ttyUSB0'
self.lidar = LPFL.RPLidar(LIDAR_PORT)
```

**수정할 부분**
- `LIDAR_PORT`는 ROS 파라미터가 아니라 파일 상단에 박힌 상수라서, 실제 장치명이 다르면 이 파일을 직접 열어서 값 교체해야 함 (`ls /dev/ttyUSB*`로 확인)

### 4.2 장착 방향 보정 (핵심)

**경로**: `src/lidar_perception_pkg/lidar_perception_pkg/lidar_processor_node.py`

```python
msg = LPFL.rotate_lidar_data(msg, offset=0)   # offset: 0~359
msg = LPFL.flip_lidar_data(msg, pivot_angle=0)  # pivot_angle: 0~359
```

라이다를 뒤에 다는 것과 관련해서 **바로 이 부분이 우리가 써야 할 기능**. `lidar_obstacle_detector_node`의 각도 범위(아래 4.3)를 건드리지 않고, 여기서 `offset` 값만 조정하면 라이다가 물리적으로 어느 방향을 보고 있든 소프트웨어상에서 "0도 = 원하는 기준 방향"이 되도록 데이터 자체를 돌려버릴 수 있음.

**수정할 부분**
- 라이다를 후방 장착한 상태에서 실제로 라이다 커넥터/마크가 가리키는 물리적 0도 방향과, 우리가 "장애물 감지 기준 정면"으로 삼고 싶은 방향 사이의 각도 차이를 실측해서 `offset` 값으로 입력

### 4.3 장애물 판단

**경로**: `src/lidar_perception_pkg/lidar_perception_pkg/lidar_obstacle_detector_node.py`

```python
start_angle = 0
end_angle = 30
range_min = 0.5   # [m]
range_max = 2.0   # [m]

detected = LPFL.detect_object(ranges=ranges, start_angle=start_angle, end_angle=end_angle,
                               range_min=range_min, range_max=range_max)

self.detection_checker = LPFL.StabilityDetector(consec_count=5)
detection_result = self.detection_checker.check_consecutive_detections(detected)
```

**수정할 부분**
- `start_angle`, `end_angle`, `range_min`, `range_max` 값을 실제 감지하고 싶은 각도·거리 범위로 직접 수정 (4.2에서 offset을 맞춰놨다면 이 각도 범위는 "차량 기준 방향" 그대로 써도 됨)
- `consec_count=5` — 5프레임 연속 감지돼야 장애물로 판정. 반응 속도 vs 오탐 방지 트레이드오프로 조정 가능
- **주의**: 이 로직(감지되면 무조건 정지)은 "주행 중 회피"에만 써야 함. 후진 주차처럼 장애물에 의도적으로 가까이 다가가야 하는 동작에는 이 로직을 그대로 쓰면 안 됨 — 아래 8번 항목 참고

---

## 5. 경로 계산 + 최종 판단

### 5.1 경로 계산

**경로**: `src/decision_making_pkg/decision_making_pkg/path_planner_node.py`

```python
CAR_CENTER_POINT = (320, 179)  # 이미지 상 차량 앞범퍼 중심 픽셀 좌표

y_points_list.append(self.car_center_point[1])
x_points_list.append(self.car_center_point[0])
...
cs = CubicSpline(y_points, x_points, bc_type='natural')
```

차선 중앙 지점(3.3에서 나온 target_points) 3개 이상 + 차량 기준점 1개를 합쳐서 CubicSpline으로 경로 곡선을 만듦.

**수정할 부분**
- `CAR_CENTER_POINT` — 우리 카메라 해상도(640x480 그대로 쓸 경우 해상도는 안 바뀌지만) 및 장착 위치 기준으로, 실제 영상에서 범퍼 중심이 찍히는 픽셀 좌표를 재측정해서 교체

### 5.2 최종 판단

**경로**: `src/decision_making_pkg/decision_making_pkg/motion_planner_node.py`

```python
if self.lidar_data is not None and self.lidar_data.data is True:
    self.steering_command, self.left_speed_command, self.right_speed_command = 0, 0, 0

elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
    for detection in self.detection_data.detections:
        if detection.class_name == 'traffic_light':
            ...정지 처리...

else:
    target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
    self.steering_command = convert_steeringangle2command(52, target_slope)
    self.left_speed_command = 80
    self.right_speed_command = 80
```

우선순위는 라이다 장애물 > 빨간 신호등 > 차선 추종 순. 조향값은 `convert_steeringangle2command`(파일 맨 아래 정의, 3차함수로 slope를 -7~7 조향 단계로 변환)로 계산하고, 속도는 좌우 모두 고정값 80.

**수정할 부분**
- `if detection.class_name=='traffic_light':` 이 부분 — 3.4에서 설명한 것처럼 지금 모델엔 이 클래스가 없어서 이 분기는 절대 안 걸림. 3.4의 대안(A)로 신호등 노드를 바꾸면, 이 신호등 정지 분기 자체는 `self.traffic_light_data.data == 'Red'` 조건만으로 처리되니(위쪽 elif 조건) 그대로 둬도 됨 — 다만 내부의 `for detection...` 루프(줄 90~101)는 `traffic_light` 클래스가 아예 안 나오는 상황에선 무의미하므로 지워도 됨
- `left_speed_command = 80`, `right_speed_command = 80` — 고정 속도값. 우리 모터 사양(전압/기어비)에 맞는 실제 주행 속도로 실측 후 조정
- `convert_steeringangle2command(52, target_slope)`의 `52`는 "이 정도 slope가 최대 조향각에 대응한다"는 기준값 — 우리 카메라 각도/트랙 곡률 기준으로 재조정 필요
- 파일 구조상 버그: 106~113번째 줄에서 `steering_command`를 계산해놓고 바로 다음 줄(115번째 줄)에서 `convert_steeringangle2command`로 덮어써서 106~113번째 줄은 실행은 되지만 결과에 영향 없는 죽은 코드임 — 정리해도 무방
- `DMFL.calculate_slope_between_points`도 3.3과 마찬가지로 `decision_making_func_lib`의 소스(.py)가 없고 `.pyc`만 있어서 내부 계산 방식을 보려면 별도로 확인 필요 (실행 자체는 문제없음)

---

## 6. 초음파 센서

레포 코드에 초음파 관련 파일 자체가 없음. 별도로 가져다 쓸 코드 없음.

---

## 7. 실행 설정 (launch 파일)

**경로**: `src/launch_pkg/launch/main.launch.py`

현재 내용은 실주행에 필요한 노드(yolov8, 차선인식, 경로계산, 시리얼전송, 최종판단)가 전부 주석 처리돼 있고, `ethernet_image_publisher_node`(이더넷 카메라 5대용, 다른 하드웨어 구성) 5개만 켜져 있음:

```python
Node(package='camera_perception_pkg', executable='ethernet_image_publisher_node',
     name='ethernet_image_publisher_node_1', parameters=[{'image': 'image_01'}]),
# ... 4개 더 (ethernet_image_publisher_node_2~5)

# Node(package='camera_perception_pkg', executable='yolov8_node', ...),          # 주석 처리됨
# Node(package='camera_perception_pkg', executable='lane_info_extractor_node', ...),  # 주석 처리됨
# Node(package='decision_making_pkg', executable='path_planner_node', ...),      # 주석 처리됨
# Node(package='serial_communication_pkg', executable='serial_sender_node', ...), # 주석 처리됨
# Node(package='decision_making_pkg', executable='motion_planner_node', ...),    # 주석 처리됨
```

**수정할 부분**
- 이 파일을 우리 구성용으로 새로 작성: `ethernet_image_publisher_node` 5개 대신 `image_publisher_node`(전방, `cam_num:=0, data_source:='camera'`) + `image_publisher_node`(후방, `cam_num:=1, pub_topic:='image_02'`)로 교체
- 주석 처리된 5개 노드(`yolov8_node`, `lane_info_extractor_node`, `path_planner_node`, `serial_sender_node`, `motion_planner_node`) 주석 해제
- `lidar_publisher_node`, `lidar_processor_node`, `lidar_obstacle_detector_node`는 launch 파일에 아예 없어서 새로 추가해야 함

---

## 8. 후진 주차 로직 (레포에 없어서 신규 작성 필요)

4.3의 `lidar_obstacle_detector_node`는 "범위 안에 뭔가 있으면 True/False"만 반환하는 이진 판단이라 일반 주행 중 회피에는 맞지만, 후진 주차처럼 장애물(벽·차량·주차선 경계)에 점점 가까이 다가가야 하는 동작에는 그대로 쓰면 안 됨. 진입하자마자 장애물이 잡히는 순간 바로 멈춰버림. 아래처럼 별도 로직이 필요함.

### 8.1 라이다: Bool 대신 실제 거리값 발행

`lidar_obstacle_detector_node`에 아래와 같은 함수를 추가해서, 기존 `lidar_obstacle_info`(Bool, 회피용)는 그대로 두고 `lidar_min_distance`(Float32, 주차용) 토픽을 하나 더 발행:

```python
import numpy as np
from std_msgs.msg import Float32

def get_min_distance_in_range(ranges, start_angle, end_angle):
    angles = np.arange(len(ranges))
    if start_angle <= end_angle:
        mask = (angles >= start_angle) & (angles <= end_angle)
    else:  # 0도를 걸치는 범위 (예: 350~10도)
        mask = (angles >= start_angle) | (angles <= end_angle)
    valid = np.array(ranges)[mask]
    valid = valid[np.isfinite(valid)]
    return float(valid.min()) if len(valid) > 0 else float('inf')
```

`ranges`는 `LaserScan` 메시지에 원래 들어있는 배열이라, `lidar_perception_func_lib`(소스 없는 `.pyc`) 없이 이 노드 안에서 바로 계산 가능.

### 8.2 최종판단: 거리 기반 비례 정지 + 주행모드 분리

`motion_planner_node`에 `parking_mode` 상태를 하나 추가해서, 이 상태일 때는 차선추종 로직 대신 아래 로직을 타도록 분기:

```python
PARKING_SLOW_DIST = 0.40   # [m] 이 거리부터 감속 시작
PARKING_STOP_DIST = 0.10   # [m] 이 거리 이하면 완전 정지

if self.parking_mode:
    d = self.lidar_min_distance  # 8.1에서 새로 만든 토픽 구독
    if d <= PARKING_STOP_DIST:
        speed = 0
    elif d >= PARKING_SLOW_DIST:
        speed = -80  # 후진 속도 (부호는 driving.ino의 setLeftMotorSpeed/setRightMotorSpeed 방향 정의에 맞춰 확인)
    else:
        ratio = (d - PARKING_STOP_DIST) / (PARKING_SLOW_DIST - PARKING_STOP_DIST)
        speed = -80 * ratio  # 가까워질수록 선형으로 감속

    self.left_speed_command = self.right_speed_command = speed
    self.steering_command = 0  # 조향은 별도 로직 필요 (8.3 참고)
```

`PARKING_SLOW_DIST`, `PARKING_STOP_DIST`는 실측 후 조정. 차체 크기·라이다 장착 위치에 따라 차이가 큼.

### 8.3 아직 레포에 없어서 직접 설계해야 하는 부분

- **주차 시작 트리거**: `parking_mode`를 언제 True로 바꿀지에 대한 코드가 레포에 없음. 특정 주차 표지판을 YOLO로 인식하거나, 경로상 특정 지점 도달, 혹은 별도 초음파/라이다 트리거 등 대회 미션 규정에 맞는 방식으로 새로 설계 필요
- **후진 중 조향**: 위 8.2 코드는 직진 후진만 가정함. 대각선/평행주차처럼 후진 중 조향이 필요하면, 목표 주차 각도까지 조향을 얼마나 꺾을지 계산하는 로직을 별도로 추가해야 함 (현재 레포의 조향 로직은 전방 차선 추종 전용이라 재사용 불가, 새로 작성)
- **주차 완료 판단**: 완전 정지 후 다음 동작(정지 유지, 다음 미션으로 전환 등)을 어떻게 트리거할지도 대회 규정에 맞춰 정의 필요
