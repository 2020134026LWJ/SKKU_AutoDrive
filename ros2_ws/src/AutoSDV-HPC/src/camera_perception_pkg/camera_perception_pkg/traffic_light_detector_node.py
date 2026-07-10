import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# 신호등 인식 (§3.4 대안 A — YOLO 우회, HSV 직접 검출)
#
# 배경: best_urp.pt 에 'traffic_light' 클래스가 없어서, YOLO bbox에 의존하던
#   기존 로직은 조건이 절대 안 걸려 항상 'None'만 발행했음.
#   → detections 구독을 버리고, 카메라 영상 ROI에 HSV inRange를 직접 걸어
#     빨강/노랑/초록 픽셀 비율로 색을 판정한다.
#
# 발행 토픽은 그대로 (motion_planner_node 가 'yolov8_traffic_light_info' 를 구독).
# ---------------------------------------------------------------------------

# Subscribe / Publish 토픽
SUB_IMAGE_TOPIC_NAME = "image_01"                 # 전방 카메라 영상 (image_publisher_node 발행)
PUB_TOPIC_NAME = "yolov8_traffic_light_info"      # motion_planner_node 가 구독

# --- 튜닝값 (하드웨어/트랙에서 조정: docs/CALIBRATION.md 참고) ---
ROI_TOP = 0.0      # TODO: 신호등이 잡히는 세로 구간 시작 비율 (0=맨위)
ROI_BOTTOM = 0.5   # TODO: 끝 비율 (0.5=위쪽 절반만 봄)
MIN_COLOR_RATIO = 0.005   # TODO: ROI 대비 해당 색 픽셀이 이 비율 넘어야 "켜짐"으로 판정 (0.5%)

# HSV 색 범위 (기존 get_traffic_light_color 와 동일 값 재사용)
HSV_RANGES = {
    'red1':   (np.array([0, 100, 95]),   np.array([10, 255, 255])),
    'red2':   (np.array([160, 100, 95]), np.array([179, 255, 255])),
    'yellow': (np.array([20, 100, 95]),  np.array([30, 255, 255])),
    'green':  (np.array([40, 100, 95]),  np.array([90, 255, 255])),
}


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.roi_top = self.declare_parameter('roi_top', ROI_TOP).value
        self.roi_bottom = self.declare_parameter('roi_bottom', ROI_BOTTOM).value
        self.min_color_ratio = self.declare_parameter('min_color_ratio', MIN_COLOR_RATIO).value

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

    def detect_color(self, cv_image):
        """ROI에서 HSV inRange로 빨강/노랑/초록 픽셀 비율을 재고 가장 강한 색을 반환."""
        h = cv_image.shape[0]
        y0 = int(h * self.roi_top)
        y1 = int(h * self.roi_bottom)
        roi = cv_image[y0:y1, :]
        if roi.size == 0:
            return 'None'

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_pixels = roi.shape[0] * roi.shape[1]

        def ratio(mask):
            return cv2.countNonZero(mask) / roi_pixels

        red = ratio(cv2.inRange(hsv, *HSV_RANGES['red1'])) + ratio(cv2.inRange(hsv, *HSV_RANGES['red2']))
        yellow = ratio(cv2.inRange(hsv, *HSV_RANGES['yellow']))
        green = ratio(cv2.inRange(hsv, *HSV_RANGES['green']))

        colors = {'Red': red, 'Yellow': yellow, 'Green': green}
        best = max(colors, key=colors.get)
        if colors[best] < self.min_color_ratio:
            return 'None'
        return best

    def image_callback(self, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        color = self.detect_color(cv_image)

        color_msg = String()
        color_msg.data = color
        self.get_logger().info(f'traffic light: {color}')
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
