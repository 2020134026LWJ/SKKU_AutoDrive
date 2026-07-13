import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib.cv_bridge_np import CvBridge   # numpy2 호환 (원본 cv_bridge는 numpy1 C확장)
from .lib.traffic_light_lib import TrafficLightParams, TrafficLightVoter, find_lamp

from sensor_msgs.msg import Image
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# 신호등 인식 (§3.4 대안 A — YOLO 우회, 영상에서 직접 검출)
#
# best_urp.pt 에 'traffic_light' 클래스가 없어서 YOLO bbox에 의존할 수 없다.
# 검출 로직은 전부 lib/traffic_light_lib.py 에 있다 (ROS 비의존) — 덕분에
# `python3 scripts/traffic_light_eval.py` 로 **차 없이** 오검출/검출률을 잰다.
#
# 실측 (2026-07-13, 녹화 영상 98초 = 신호등이 하나도 없는 실내 로비):
#   기존 '색 비율' 로직 : 71% 프레임에서 신호등 색 오검출 (빨강 55% / 초록 16%)
#   현재 blob 로직      : 오검출 0%, 합성 신호등 검출률 99.9%
#
# 발행 토픽은 그대로 (motion_planner_node 가 'yolov8_traffic_light_info' 구독).
# ---------------------------------------------------------------------------

SUB_IMAGE_TOPIC_NAME = "image_01"                 # 전방 카메라 영상
PUB_TOPIC_NAME = "yolov8_traffic_light_info"      # motion_planner_node 가 구독


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # 검출 파라미터 — 전부 ROS 파라미터로 override 가능 (docs/CALIBRATION.md).
        # 실물에서 조정할 것은 사실상 min_area/max_area 둘 (= 신호등이 화면에서 얼마나 크게
        # 보이나). 나머지는 '램프의 생김새'라서 트랙이 바뀌어도 잘 안 변한다.
        d = TrafficLightParams()
        self.params = TrafficLightParams(
            roi_top=self.declare_parameter('roi_top', d.roi_top).value,
            roi_bottom=self.declare_parameter('roi_bottom', d.roi_bottom).value,
            sat_min=self.declare_parameter('sat_min', d.sat_min).value,
            val_min=self.declare_parameter('val_min', d.val_min).value,
            min_area=self.declare_parameter('min_area', d.min_area).value,
            max_area=self.declare_parameter('max_area', d.max_area).value,
            min_circularity=self.declare_parameter('min_circularity', d.min_circularity).value,
            min_fill_ratio=self.declare_parameter('min_fill_ratio', d.min_fill_ratio).value,
            max_aspect=self.declare_parameter('max_aspect', d.max_aspect).value,
            ring_ratio=self.declare_parameter('ring_ratio', d.ring_ratio).value,
            max_surround_ratio=self.declare_parameter('max_surround_ratio', d.max_surround_ratio).value,
            consec_frames=self.declare_parameter('consec_frames', d.consec_frames).value,
        )
        self.voter = TrafficLightVoter(self.params.consec_frames)

        self.cv_bridge = CvBridge()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image, self.sub_image_topic, self.image_callback, self.qos_profile)
        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

        self._last_logged = None

    def image_callback(self, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        color, bbox = find_lamp(cv_image, self.params)
        state = self.voter.update(color)     # N프레임 연속 같은 색일 때만 확정

        # 상태가 바뀔 때만 로그 — 30Hz로 매 프레임 찍으면 터미널이 덮인다
        if state != self._last_logged:
            self.get_logger().info(f'traffic light: {self._last_logged} → {state}'
                                   + (f' (bbox={bbox})' if bbox else ''))
            self._last_logged = state

        color_msg = String()
        color_msg.data = state
        self.publisher.publish(color_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
