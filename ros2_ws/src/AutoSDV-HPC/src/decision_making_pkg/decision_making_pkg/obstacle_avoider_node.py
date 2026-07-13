import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String
from interfaces_pkg.msg import DetectionArray, MotionCommand

from .lib.avoid_lib import AvoidController, AvoidParams

# ---------------------------------------------------------------------------
# 장애물 회피 (차선 변경) — 미션 1.
#
#   detections(YOLO) ─→ [이 노드] ─→ target_lane (String: "lane1" | "lane2")
#                                          ↓
#                            lane_info_extractor_node 가 그 차선을 따라간다
#                                          ↓
#                            path_planner → motion_planner → 모터
#
# [핵심] 이 노드는 **조향값을 만들지 않는다.** "어느 차선을 따라갈지"만 바꾼다.
# 회피가 직접 핸들을 꺾으면 차선 추종과 싸우게 된다. 목표 차선만 바꾸면 기존
# 경로계획이 알아서 부드러운 곡선을 그려준다.
# (제어 토픽 발행자를 motion_planner 하나로 유지 — 주차 노드와 같은 원칙)
#
# 판단 로직은 lib/avoid_lib.py (ROS 비의존) — `scripts/avoid_eval.py`로 차 없이 채점.
# ---------------------------------------------------------------------------

SUB_DETECTION_TOPIC_NAME = "detections"
SUB_CONTROL_TOPIC_NAME = "topic_control_signal"   # 내가 실제로 얼마나 갔나(속도 적분)
PUB_LANE_TOPIC_NAME = "target_lane"
PUB_STATE_TOPIC_NAME = "avoid_state"              # 디버그용

TIMER = 0.1   # 거리 적분 주기 [s]


class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_control_topic = self.declare_parameter('sub_control_topic', SUB_CONTROL_TOPIC_NAME).value
        self.pub_lane_topic = self.declare_parameter('pub_lane_topic', PUB_LANE_TOPIC_NAME).value
        self.pub_state_topic = self.declare_parameter('pub_state_topic', PUB_STATE_TOPIC_NAME).value
        self.timer_period = self.declare_parameter('timer', TIMER).value

        d = AvoidParams()
        self.params = AvoidParams(
            home_lane=self.declare_parameter('home_lane', d.home_lane).value,
            other_lane=self.declare_parameter('other_lane', d.other_lane).value,
            obstacle_class=self.declare_parameter('obstacle_class', d.obstacle_class).value,
            img_width=self.declare_parameter('img_width', d.img_width).value,
            img_height=self.declare_parameter('img_height', d.img_height).value,
            trigger_y=self.declare_parameter('trigger_y', d.trigger_y).value,
            min_score=self.declare_parameter('min_score', d.min_score).value,
            lane_half_width=self.declare_parameter('lane_half_width', d.lane_half_width).value,
            clear_frames=self.declare_parameter('clear_frames', d.clear_frames).value,
            pass_clearance_m=self.declare_parameter('pass_clearance_m', d.pass_clearance_m).value,
            trigger_frames=self.declare_parameter('trigger_frames', d.trigger_frames).value,
        )
        # 주차와 같은 값 (PWM 1당 m/s). 오도메트리가 없어서 자기가 낸 속도로 거리를 추정한다.
        self.speed_to_mps = self.declare_parameter('speed_to_mps', 0.003).value

        self.controller = AvoidController(self.params)
        self.speed = 0

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.create_subscription(DetectionArray, self.sub_detection_topic,
                                 self.detection_callback, self.qos_profile)
        self.create_subscription(MotionCommand, self.sub_control_topic,
                                 self.control_callback, self.qos_profile)

        self.lane_pub = self.create_publisher(String, self.pub_lane_topic, self.qos_profile)
        self.state_pub = self.create_publisher(String, self.pub_state_topic, self.qos_profile)

        # 목표 차선은 **계속** 발행한다. 한 번만 쏘면 lane_info_extractor가 아직 뜨기
        # 전이면 유실되고, 그러면 영원히 기본 차선만 따라간다. (주차 트리거에서 겪었다)
        self.create_timer(self.timer_period, self.timer_callback)

        self._last_state = None

    def control_callback(self, msg: MotionCommand):
        self.speed = msg.left_speed

    def detection_callback(self, msg: DetectionArray):
        state, lane = self.controller.update(msg.detections)
        if state != self._last_state:
            self.get_logger().info(f'회피: {self._last_state} -> {state} (따라갈 차선: {lane})')
            self._last_state = state

    def timer_callback(self):
        # 전진한 거리를 누적 (AVOID 중일 때만 의미가 있다 — 장애물을 완전히 지나쳤나)
        if self.speed > 0:
            self.controller.add_travel(self.speed * self.speed_to_mps * self.timer_period)

        self.lane_pub.publish(String(data=self.controller.target_lane))
        self.state_pub.publish(String(data=self.controller.state))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
