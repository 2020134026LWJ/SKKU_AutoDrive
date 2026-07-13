import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib.cv_bridge_np import CvBridge   # numpy2 호환 (원본 cv_bridge는 numpy1 C확장)
from .lib.rear_park_lib import RearParkParams, find_lateral_error, find_rear_line

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# ---------------------------------------------------------------------------
# 후방 카메라 주차 인식 — 주차 FSM의 '눈'.
#
#   image_02 (후방 영상) ─→ [이 노드] ─→ parking_rear_distance (Float32)
#                                     └─→ parking_lateral_error  (Float32)
#
# 원래 주차는 후방 라이다가 '뒷벽'까지의 거리를 줬다. 그런데 실제 주차장엔 뒷벽이 없고
# 양옆에 차만 있다(2026-07-13 확인) → 멈출 근거를 '주차칸 뒤 경계선'으로 바꿨다.
# parking_controller_node는 Float32 하나만 소비하므로 출처가 바뀐 걸 모른다.
#
# [중요] 뒷선이 안 보이면 **아무것도 발행하지 않는다.**
# 억지로 추정해서 내보내면 FSM이 그 거짓값을 믿고 계속 후진한다. 침묵이 정답이고,
# FSM은 그 침묵을 상황에 따라 해석한다 (아직 못 봄 / 사각지대 = 다 들어옴 / 고장).
# 검출 로직은 lib/rear_park_lib.py (ROS 비의존) — `scripts/rear_park_eval.py`로 차 없이 채점.
# ---------------------------------------------------------------------------

SUB_IMAGE_TOPIC_NAME = "image_02"              # 후방 카메라 (image_publisher_node를 하나 더 띄운다)
PUB_DISTANCE_TOPIC_NAME = "parking_rear_distance"
PUB_LATERAL_TOPIC_NAME = "parking_lateral_error"


class RearParkDetector(Node):
    def __init__(self):
        super().__init__('rear_park_detector_node')

        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_distance_topic = self.declare_parameter('pub_distance_topic', PUB_DISTANCE_TOPIC_NAME).value
        self.pub_lateral_topic = self.declare_parameter('pub_lateral_topic', PUB_LATERAL_TOPIC_NAME).value

        d = RearParkParams()
        # src_mat/dst_mat은 4점 x 2좌표 = flat 8개로 받는다 (ROS 파라미터가 중첩 배열을 못 받음)
        src_flat = self.declare_parameter(
            'src_mat', [float(v) for pt in d.src_mat for v in pt]).value
        dst_flat = self.declare_parameter(
            'dst_mat', [float(v) for pt in d.dst_mat for v in pt]).value

        self.params = RearParkParams(
            src_mat=[(src_flat[i], src_flat[i + 1]) for i in range(0, 8, 2)],
            dst_mat=[(dst_flat[i], dst_flat[i + 1]) for i in range(0, 8, 2)],
            m_per_px=self.declare_parameter('m_per_px', d.m_per_px).value,
            cam_offset_m=self.declare_parameter('cam_offset_m', d.cam_offset_m).value,
            center_band=self.declare_parameter('center_band', d.center_band).value,
            white_min=self.declare_parameter('white_min', d.white_min).value,
            sat_max=self.declare_parameter('sat_max', d.sat_max).value,
            min_line_px=self.declare_parameter('min_line_px', d.min_line_px).value,
            min_line_ratio=self.declare_parameter('min_line_ratio', d.min_line_ratio).value,
            max_line_thickness=self.declare_parameter('max_line_thickness', d.max_line_thickness).value,
            side_darker_than=self.declare_parameter('side_darker_than', d.side_darker_than).value,
            min_slot_cols=self.declare_parameter('min_slot_cols', d.min_slot_cols).value,
            min_side_cols=self.declare_parameter('min_side_cols', d.min_side_cols).value,
        )

        self.cv_bridge = CvBridge()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image, self.sub_image_topic, self.image_callback, self.qos_profile)
        self.distance_pub = self.create_publisher(Float32, self.pub_distance_topic, self.qos_profile)
        self.lateral_pub = self.create_publisher(Float32, self.pub_lateral_topic, self.qos_profile)

        self._was_visible = None

    def image_callback(self, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        distance = find_rear_line(cv_image, self.params)
        if distance is not None:
            self.distance_pub.publish(Float32(data=float(distance)))
        # else: 발행하지 않는다 (침묵). FSM이 '안 보임'을 스스로 해석한다.

        if distance is not None and self._was_visible is not True:
            self.get_logger().info(f'뒷선 확보 — d={distance:.3f}m')
            self._was_visible = True
        elif distance is None and self._was_visible is not False:
            self.get_logger().info('뒷선 안 보임 — 발행 중단 (FSM이 판단한다)')
            self._was_visible = False

        lateral = find_lateral_error(cv_image, self.params)
        if lateral is not None:
            self.lateral_pub.publish(Float32(data=float(lateral)))


def main(args=None):
    rclpy.init(args=args)
    node = RearParkDetector()
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
