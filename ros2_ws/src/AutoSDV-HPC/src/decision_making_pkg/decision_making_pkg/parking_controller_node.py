"""직각(후진) 주차 FSM.

주행 중 판단(motion_planner_node)과 분리된 별도 노드다. 주차는 본질적으로 다단계
시퀀스(정렬 → 조향후진 → 직진후진 → 정지)라, motion_planner의 상태 없는 if/elif
분기에 끼워넣으면 유지보수가 안 된다.

제어 토픽(topic_control_signal)에 직접 쓰지 않는다. 발행자가 둘이 되면 명령이 서로
덮어써서 원인 추적이 불가능해진다. 대신 parking_active/parking_command를 내보내고,
motion_planner가 active일 때 양보(relay)한다.

    parking_trigger(Bool) ──┐
    parking_rear_distance ──┴→ [이 노드] → parking_active(Bool)
                                         → parking_command(MotionCommand)
                                         → parking_state(String, 디버그용)

[중요] 이 차량은 오도메트리도 IMU도 없다. 즉 "90도 돌았는지"를 측정할 방법이 없어서
진입 회전은 시간 기반 개루프다. 아래 duration 파라미터들은 전부 실차에서 재야 하는
값이고, 지금 박혀있는 숫자는 자리를 채운 것일 뿐 의미 없다. 실측 절차는
docs/CALIBRATION.md 참고.
최종 접근만 거리 기반 폐루프라서 개루프 오차를 흡수한다.

[거리의 출처] 원래 후방 라이다가 '뒷벽'까지의 거리를 줬다. 그런데 실제 주차장엔
**뒷벽이 없고 양옆에 차만 있다**(2026-07-13 확인). 그래서 거리 출처를 후방 카메라가
재는 '뒤 경계선까지의 거리'로 바꿨다. 이 노드는 출처를 모른다 — Float32 하나만
소비하므로 라이다든 카메라든 그대로 돈다. 대신 카메라는 **안 보이는 구간**이 있어서
그 처리가 들어갔다 (아래 BLIND/SEARCH 참고).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Bool, Float32, String
from interfaces_pkg.msg import MotionCommand

#---------------Variable Setting---------------
SUB_TRIGGER_TOPIC_NAME = "parking_trigger"
SUB_DISTANCE_TOPIC_NAME = "parking_rear_distance"  # 후방 카메라가 재는 "뒤 경계선까지 거리". 라이다에서 교체됨
PUB_ACTIVE_TOPIC_NAME = "parking_active"
PUB_COMMAND_TOPIC_NAME = "parking_command"
PUB_STATE_TOPIC_NAME = "parking_state"

TIMER = 0.1  # FSM 갱신 주기 [s]. motion_planner와 동일하게 10Hz.

# --- 실측 필요 상수 (전부 ROS 파라미터로 override 가능) ---
ALIGN_DURATION = 1.0      # [s] 주차칸을 지나쳐 정렬 위치까지 전진하는 시간
ALIGN_SPEED = 60          # 정렬 전진 속도 (PWM, 0~255)

TURN_DURATION = 2.5       # [s] 조향 최대로 꺾고 후진하는 시간 (≈90도 회전)
REVERSE_SPEED = 70        # 후진 속도 (PWM). 음수 부호는 코드가 붙인다.

STEER_MAX = 7             # driving.ino의 MAX_STEERING_STEP과 반드시 일치
SLOT_SIDE = "right"       # 주차칸이 차 기준 어느 쪽인지 ("left" | "right")

PARKING_SLOW_DIST = 0.40  # [m] 이 거리부터 감속 시작
PARKING_STOP_DIST = 0.10  # [m] 이 거리 이하면 완전 정지
MIN_MOVE_SPEED = 45       # 이 PWM 아래로는 모터가 안 돎(데드밴드). 비례감속의 하한.

DISTANCE_TIMEOUT = 0.3    # [s] 거리값이 이만큼 조용하면 '끊긴' 것으로 본다.
                          # 라이다(10Hz) 기준 1.0s였는데 카메라(30Hz)엔 너무 느리다 —
                          # 끊긴 걸 늦게 알아챌수록 그동안 계속 후진한다.

# --- 뒷선이 안 보이는 구간 다루기 (2026-07-13) ---
# 거리 출처가 후방 카메라(뒤 경계선)로 바뀌면서, "안 보임"이 세 가지 뜻을 갖게 됐다.
# 뭉뚱그려 ABORT하면 **정상적으로 주차가 끝나갈 때마다 중단**된다.
#
#   1) 아직 못 봤다      — 칸 옆에서 비스듬히 볼 땐 옆차가 뒷선을 가린다.
#                          칸 축과 나란해져야 보인다 → 천천히 후진하며 기다린다 (SEARCH)
#   2) 가까이서 잃었다   — 범퍼 밑 사각지대로 들어간 것 = **다 들어왔다는 신호**.
#                          남은 거리만큼 눈감고 밀어넣고 멈춘다 (BLIND_CREEP)
#   3) 멀리서 잃었다     — 진짜 고장(카메라 죽음/선 못 찾음) → ABORT
BLIND_ENTER_DIST = 0.30   # [m] 이 거리 안쪽에서 선을 잃으면 '사각지대'로 본다.
                          # [중요] 실제 사각지대 거리보다 **약간 크게** 잡아라.
                          # 마지막으로 관측되는 거리는 사각지대 문턱보다 항상 살짝
                          # 바깥이다(선이 사라지기 직전 프레임). 문턱과 똑같이 맞추면
                          # 정상 주차를 "멀리서 잃었다"로 오판해 ABORT한다. (실측 확인)

# 눈감고 가는 구간은 '시간'이 아니라 '거리'로 묶는다.
#
# 처음엔 시간(초)으로 짰다가 시뮬에서 걸렸다: 옆차 가림을 견디려고 탐색 시간을 늘렸더니,
# 뒷선을 끝내 못 본 시나리오에서 **주차칸을 뚫고 나갔다.** 시간은 안전의 단위가 아니다.
#
# 오도메트리는 없지만 **차는 자기가 내보낸 속도를 안다.** 그걸 적분하면 눈감고 몇 m를
# 갔는지 나온다. 필요한 건 PWM→m/s 환산상수 하나뿐이고, 그건 한 번 재면 되는 값이다.
SPEED_TO_MPS = 0.003      # [m/s per PWM] 실측 필수. 이 값이 틀리면 눈감은 구간이 전부 틀린다.
                          # 재는 법: 후진 PWM 고정하고 3초간 굴린 거리 / (PWM x 3)
BLIND_SEARCH_MAX_DIST = 0.50  # [m] 뒷선을 한 번도 못 본 채 이만큼 후진하면 ABORT.
                              # 두 조건 사이에서 잡는다:
                              #   하한 — 옆차 가림이 풀릴 때까지 갈 거리보다는 길어야 한다
                              #   상한 — **주차칸 깊이보다는 짧아야** 뚫고 나가지 않는다
                              # (상한을 어기면 never_seen에서 칸을 관통한다. 시뮬로 확인)
BLIND_FINAL_GAP = 0.05    # [m] 사각지대 마무리 후 뒷선까지 남길 여유. 0이면 닿는다.
BLIND_CREEP_MAX_SECS = 3.0    # [s] 거리 적분이 틀렸을 때를 대비한 상한 (2중 안전망)
DISTANCE_GRACE = 0.5      # [s] 멀리서 잠깐 끊긴 건 봐준다 (한 프레임 놓침 등)

PARKING_TIMEOUT = 25.0    # [s] 전체 주차 제한시간. 넘으면 ABORT.
#----------------------------------------------

# FSM 상태
IDLE = "IDLE"                        # 대기. 트리거를 기다린다.
ALIGN = "ALIGN"                      # 정렬 전진
REVERSE_TURN = "REVERSE_TURN"        # 조향 최대 + 후진 (차 뒤를 주차칸으로 밀어넣음)
REVERSE_STRAIGHT = "REVERSE_STRAIGHT"  # 조향 중립 + 후진, 뒷선 거리로 비례감속
BLIND_CREEP = "BLIND_CREEP"          # 뒷선이 사각지대로 사라짐 → 저속으로 마무리
DONE = "DONE"                        # 주차 완료. 정지 유지.
ABORT = "ABORT"                      # 이상 상황. 정지 유지.


class ParkingControllerNode(Node):
    def __init__(self):
        super().__init__('parking_controller_node')

        self.sub_trigger_topic = self.declare_parameter('sub_trigger_topic', SUB_TRIGGER_TOPIC_NAME).value
        self.sub_distance_topic = self.declare_parameter('sub_distance_topic', SUB_DISTANCE_TOPIC_NAME).value
        self.pub_active_topic = self.declare_parameter('pub_active_topic', PUB_ACTIVE_TOPIC_NAME).value
        self.pub_command_topic = self.declare_parameter('pub_command_topic', PUB_COMMAND_TOPIC_NAME).value
        self.pub_state_topic = self.declare_parameter('pub_state_topic', PUB_STATE_TOPIC_NAME).value
        self.timer_period = self.declare_parameter('timer', TIMER).value

        self.align_duration = self.declare_parameter('align_duration', ALIGN_DURATION).value
        self.align_speed = self.declare_parameter('align_speed', ALIGN_SPEED).value
        self.turn_duration = self.declare_parameter('turn_duration', TURN_DURATION).value
        self.reverse_speed = self.declare_parameter('reverse_speed', REVERSE_SPEED).value
        self.steer_max = self.declare_parameter('steer_max', STEER_MAX).value
        self.slot_side = self.declare_parameter('slot_side', SLOT_SIDE).value
        self.slow_dist = self.declare_parameter('parking_slow_dist', PARKING_SLOW_DIST).value
        self.stop_dist = self.declare_parameter('parking_stop_dist', PARKING_STOP_DIST).value
        self.min_move_speed = self.declare_parameter('min_move_speed', MIN_MOVE_SPEED).value
        self.distance_timeout = self.declare_parameter('distance_timeout', DISTANCE_TIMEOUT).value
        self.blind_enter_dist = self.declare_parameter('blind_enter_dist', BLIND_ENTER_DIST).value
        self.speed_to_mps = self.declare_parameter('speed_to_mps', SPEED_TO_MPS).value
        self.blind_search_max_dist = self.declare_parameter('blind_search_max_dist', BLIND_SEARCH_MAX_DIST).value
        self.blind_final_gap = self.declare_parameter('blind_final_gap', BLIND_FINAL_GAP).value
        self.blind_creep_max_secs = self.declare_parameter('blind_creep_max_secs', BLIND_CREEP_MAX_SECS).value
        self.distance_grace = self.declare_parameter('distance_grace', DISTANCE_GRACE).value
        self.parking_timeout = self.declare_parameter('parking_timeout', PARKING_TIMEOUT).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.state = IDLE
        self.state_entered_at = self.now()
        self.parking_started_at = None
        self.trigger = False
        self.min_distance = None
        self.distance_stamp = None
        self.last_seen_distance = None   # 마지막으로 '실제로 본' 거리 (사각지대 판정용)
        self.blind_travel = 0.0          # 뒷선을 못 보는 동안 눈감고 후진한 거리 [m] (속도 적분)

        self.create_subscription(Bool, self.sub_trigger_topic, self.trigger_callback, self.qos_profile)
        self.create_subscription(Float32, self.sub_distance_topic, self.distance_callback, self.qos_profile)

        self.active_pub = self.create_publisher(Bool, self.pub_active_topic, self.qos_profile)
        self.command_pub = self.create_publisher(MotionCommand, self.pub_command_topic, self.qos_profile)
        self.state_pub = self.create_publisher(String, self.pub_state_topic, self.qos_profile)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # --- 유틸 ---

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_in_state(self):
        return self.now() - self.state_entered_at

    def transition(self, new_state, reason=""):
        if new_state == self.state:
            return
        self.get_logger().info(f'parking: {self.state} -> {new_state} {reason}')
        self.state = new_state
        self.state_entered_at = self.now()

    def steer_into_slot(self):
        """주차칸 쪽으로 최대 조향.

        [실측 필요] 후진 중에는 차체가 조향 방향의 반대로 돈다. 아래 부호가 실제로
        어느 쪽으로 꺾는지는 조향 모터 배선(driving.ino의 STEERING_1/2)과 가변저항
        방향에 달려 있어서, 차를 띄워놓고 한 번 돌려봐야 확정된다.
        반대로 돌면 이 부호만 뒤집으면 된다.
        """
        sign = 1 if self.slot_side == "right" else -1
        return sign * self.steer_max

    def is_distance_fresh(self):
        if self.min_distance is None or self.distance_stamp is None:
            return False
        return (self.now() - self.distance_stamp) <= self.distance_timeout

    def approach_speed(self):
        """라이다 거리 기반 비례 감속. 후진이므로 음수를 돌려준다."""
        d = self.min_distance

        if d <= self.stop_dist:
            return 0
        if d >= self.slow_dist:
            return -int(self.reverse_speed)

        ratio = (d - self.stop_dist) / (self.slow_dist - self.stop_dist)
        speed = self.reverse_speed * ratio
        # 데드밴드: 이 아래로는 PWM을 줘도 모터가 안 돌아서 "느리게 가는" 게 아니라
        # 그냥 멈춘다. 정지 거리에 닿기 전까지는 최소 구동력을 유지한다.
        speed = max(speed, self.min_move_speed)
        return -int(round(speed))

    # --- 콜백 ---

    def trigger_callback(self, msg: Bool):
        self.trigger = msg.data

    def distance_callback(self, msg: Float32):
        self.min_distance = msg.data
        self.distance_stamp = self.now()
        self.last_seen_distance = msg.data   # 끊긴 뒤 '어디서 잃었나'를 판정하는 근거
        self.blind_travel = 0.0              # 뒷선이 보인다 = 폐루프. 눈감은 거리 리셋.

    def accumulate_blind_travel(self, speed):
        """눈감고 후진한 거리를 속도 적분으로 추정 [m].

        오도메트리가 없어도 **자기가 내보낸 속도는 안다.** 뒷선을 못 보는 구간
        (탐색/사각지대)의 안전은 전부 이 값에 걸려 있다 — 시간으로 재면 속도가
        바뀔 때 거리가 달라져서 주차칸을 뚫고 나간다(시뮬에서 실제로 뚫었다).
        """
        self.blind_travel += abs(speed) * self.speed_to_mps * self.timer_period

    # --- FSM ---

    def timer_callback(self):
        steering, speed = self.step()

        # blind_travel = **마지막으로 뒷선을 본 순간부터** 후진한 거리.
        # (distance_callback이 관측 때마다 0으로 리셋하므로 자동으로 그 뜻이 된다)
        #
        # [중요] 신선도(is_distance_fresh)로 거르지 않는다. 거르면 "선이 사라졌지만 아직
        # 끊겼다고 판정하기 전"인 0.3초 구간의 이동(약 4cm)을 아무도 안 센다. 그 4cm가
        # 마지막에 남길 여유(5cm)를 거의 다 먹어서 차가 선에 닿았다. (시뮬에서 1mm 남음)
        #
        # [주의] REVERSE_TURN은 제외한다. 조향 후진은 자기 시간 제한(turn_duration)으로
        # 관리되는 별개 구간인데, 여기서 같이 세면 그 0.5m가 탐색 예산을 미리 다 써버려서
        # REVERSE_STRAIGHT에 들어서자마자 ABORT한다. (시뮬에서 실제로 그랬다)
        if self.state in (REVERSE_STRAIGHT, BLIND_CREEP) and speed < 0:
            self.accumulate_blind_travel(speed)

        active = self.state != IDLE

        command = MotionCommand()
        command.steering = int(steering)
        command.left_speed = int(speed)
        command.right_speed = int(speed)

        self.active_pub.publish(Bool(data=active))
        self.command_pub.publish(command)
        self.state_pub.publish(String(data=self.state))

    def step(self):
        """현재 상태에 맞는 (steering, speed)를 돌려주고, 필요하면 상태를 전이한다."""

        # 트리거가 내려가면 언제든 초기화 (수동 중단 / 다음 시도 준비)
        if not self.trigger and self.state != IDLE:
            self.transition(IDLE, "(트리거 해제)")
            self.parking_started_at = None
            self.last_seen_distance = None   # 다음 시도는 '아직 못 봤다'에서 시작
            self.blind_travel = 0.0
            return 0, 0

        if self.state == IDLE:
            if self.trigger:
                self.parking_started_at = self.now()
                self.transition(ALIGN, "(트리거 감지)")
            return 0, 0

        # 전체 제한시간 초과 → 정지. 개루프 구간이 있어서 무한정 굴러가는 걸 막는 안전망.
        if self.parking_started_at is not None and self.state not in (DONE, ABORT):
            if (self.now() - self.parking_started_at) > self.parking_timeout:
                self.transition(ABORT, f"(제한시간 {self.parking_timeout}s 초과)")
                return 0, 0

        if self.state == ALIGN:
            if self.elapsed_in_state() >= self.align_duration:
                self.transition(REVERSE_TURN)
                return 0, 0
            return 0, int(self.align_speed)

        if self.state == REVERSE_TURN:
            # 개루프 구간. 이미 벽에 닿을 만큼 붙었으면 시간이 남았어도 넘어간다.
            if self.is_distance_fresh() and self.min_distance <= self.stop_dist:
                self.transition(DONE, "(회전 중 정지거리 도달)")
                return 0, 0
            if self.elapsed_in_state() >= self.turn_duration:
                self.transition(REVERSE_STRAIGHT)
                self.blind_travel = 0.0     # 탐색 예산은 여기서부터 센다
                return 0, 0
            return self.steer_into_slot(), -int(self.reverse_speed)

        if self.state == REVERSE_STRAIGHT:
            if self.is_distance_fresh():
                speed = self.approach_speed()
                if speed == 0:
                    self.transition(DONE, f"(정지거리 도달, d={self.min_distance:.3f}m)")
                    return 0, 0
                return 0, speed

            # --- 뒷선이 안 보인다. 세 가지 중 뭔지 가른다 (뭉뚱그려 ABORT하면 안 된다) ---

            if self.last_seen_distance is None:
                # (1) 아직 한 번도 못 봤다 — 옆차가 가리는 중일 수 있다. 칸 축과
                #     나란해지면 보인다. 그때까지 천천히 후진하며 기다린다.
                #     [안전] 시간이 아니라 **거리**로 끊는다. 시간으로 끊으면 속도에 따라
                #     실제 이동거리가 달라져 주차칸을 뚫고 나간다.
                if self.blind_travel >= self.blind_search_max_dist:
                    self.transition(ABORT, f"(뒷선을 못 본 채 {self.blind_travel:.2f}m 후진 — 중단)")
                    return 0, 0
                return 0, -int(self.min_move_speed)

            if self.last_seen_distance <= self.blind_enter_dist:
                # (2) 가까이서 잃었다 = 범퍼 밑 사각지대로 들어갔다 = 다 들어왔다.
                #     남은 거리는 짧으니 눈감고 밀어넣는다. 개루프지만 한 뼘이라 오차가 안 쌓인다.
                # blind_travel을 리셋하지 않는다 — 이미 '마지막 관측 이후 이동거리'라서
                # 그대로 이어 세야 남은 거리가 맞는다.
                self.transition(BLIND_CREEP,
                                f"(사각지대 진입, 마지막 관측 d={self.last_seen_distance:.3f}m, "
                                f"그 뒤 {self.blind_travel:.3f}m 이동)")
                return 0, 0

            # (3) 멀리서 잃었다 = 진짜 고장. 다만 한 프레임 놓친 정도는 봐준다.
            lost_for = self.now() - self.distance_stamp
            if lost_for >= self.distance_grace:
                self.transition(ABORT,
                                f"(뒷선 끊김 — 마지막 관측 d={self.last_seen_distance:.3f}m, "
                                f"사각지대 기준 {self.blind_enter_dist}m보다 멀다)")
                return 0, 0
            return 0, -int(self.min_move_speed)   # 유예 중: 최저속으로만

        if self.state == BLIND_CREEP:
            # 눈감고 마무리. 이 구간에서 뒷선이 다시 보이면(사각지대를 잘못 짚었다면)
            # 그 값을 다시 믿는다 — 폐루프로 돌아가는 게 항상 낫다.
            if self.is_distance_fresh() and self.min_distance > self.blind_enter_dist:
                self.transition(REVERSE_STRAIGHT, "(뒷선 재확보)")
                return 0, 0

            # 갈 거리 = 마지막으로 본 거리 - 남길 여유. 여유를 안 두면 선에 닿는다.
            target = max(0.0, self.last_seen_distance - self.blind_final_gap)
            if self.blind_travel >= target:
                self.transition(DONE, f"(사각지대 마무리, 눈감고 {self.blind_travel:.3f}m 이동)")
                return 0, 0
            if self.elapsed_in_state() >= self.blind_creep_max_secs:
                # 거리 적분이 틀렸다는 뜻(speed_to_mps 오차/모터 미동작). 2중 안전망.
                self.transition(DONE, f"(사각지대 시간 상한 {self.blind_creep_max_secs}s 도달)")
                return 0, 0
            return 0, -int(self.min_move_speed)

        # DONE / ABORT: 정지 유지. 트리거를 내려야 IDLE로 돌아간다.
        return 0, 0


def main(args=None):
    rclpy.init(args=args)
    node = ParkingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
