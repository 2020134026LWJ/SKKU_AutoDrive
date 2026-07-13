import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib.cv_bridge_np import CvBridge   # numpy2 호환 (원본 cv_bridge는 numpy1 C확장)

from sensor_msgs.msg import Image
from std_msgs.msg import String
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray, BoundingBox2D, Detection
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"
SUB_TARGET_LANE_TOPIC_NAME = "target_lane"   # 회피 노드가 "이 차선을 따라가라"고 알려준다

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_lane_info"
ROI_IMAGE_TOPIC_NAME = "roi_image"  # 추가: ROI 이미지 퍼블리시 토픽

# 기본으로 따라갈 차선 (YOLO 모델 클래스: lane1 / lane2).
# 회피 노드가 target_lane 토픽으로 바꿔준다. 아무도 안 바꾸면 이 값 그대로 = 기존 동작.
DEFAULT_TARGET_LANE = "lane2"

# 버드아이뷰 4점 — 지금 값은 **원본 팀 카메라 기준**이라 우리 카메라로 100% 다시 잡아야 한다.
#   실측은 config/calibration.yaml 에서 하고 코드는 안 건드린다 (docs/CALIBRATION.md 4단계).
DEFAULT_SRC_MAT = [(238, 316), (402, 313), (501, 476), (155, 476)]
CUTTING_IDX = 250          # 버드아이뷰에서 위쪽 몇 픽셀을 버릴지 (멀리는 볼 필요 없다)
LANE_WIDTH = 300           # [px] 한쪽 차선만 보일 때 반대쪽까지의 거리 추정값. cm 아님.
DETECTION_THICKNESS = 10   # 차선 중앙을 찾을 때 훑는 띠 두께 [px]

# 처리 과정을 화면에 띄울지 (디버그 창 3개 + roi_image 토픽 발행).
#
# 기본 False — 둘 다 프레임마다 비용이 붙는다. imshow는 화면이 없으면 무의미하고,
# roi_image 토픽은 **큰 이미지를 DDS로 한 벌 더 흘려보낸다** (예전에 이미지가 커널 UDP
# 버퍼를 넘겨 프레임 80%가 유실된 적이 있다 — docs/RUNTIME_FIXES.md).
#
# 캘리브레이션할 때만 켠다 (버드아이뷰 4점 잡을 땐 오히려 꼭 봐야 한다):
#   ros2 run camera_perception_pkg lane_info_extractor_node --ros-args -p show_image:=true
SHOW_IMAGE = False
#----------------------------------------------


class Yolov8InfoExtractor(Node):
    def __init__(self):
        super().__init__('lane_info_extractor_node')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.sub_target_lane_topic = self.declare_parameter(
            'sub_target_lane_topic', SUB_TARGET_LANE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.show_image = self.declare_parameter('show_image', SHOW_IMAGE).value
        self.target_lane = self.declare_parameter('target_lane', DEFAULT_TARGET_LANE).value

        # 버드아이뷰 4점 — 카메라를 달 때마다 다시 잡아야 하는 값이라 파라미터로 뺐다.
        # (ROS 파라미터는 중첩 배열을 못 받으므로 [x1,y1,x2,y2,x3,y3,x4,y4] 8개로 받는다)
        src_flat = self.declare_parameter(
            'src_mat', [float(v) for pt in DEFAULT_SRC_MAT for v in pt]).value
        self.src_mat = [[src_flat[i], src_flat[i + 1]] for i in range(0, 8, 2)]

        self.cutting_idx = self.declare_parameter('cutting_idx', CUTTING_IDX).value
        self.lane_width = self.declare_parameter('lane_width', LANE_WIDTH).value
        self.detection_thickness = self.declare_parameter(
            'detection_thickness', DETECTION_THICKNESS).value

        self.cv_bridge = CvBridge()

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscriber = self.create_subscription(DetectionArray, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.create_subscription(String, self.sub_target_lane_topic,
                                 self.target_lane_callback, self.qos_profile)
        self.publisher = self.create_publisher(LaneInfo, self.pub_topic, self.qos_profile)

        # ROI 이미지 퍼블리셔 추가
        self.roi_image_publisher = self.create_publisher(Image, ROI_IMAGE_TOPIC_NAME, self.qos_profile)

    def target_lane_callback(self, msg: String):
        """따라갈 차선을 밖에서 바꿀 수 있게 한다 (장애물 회피 = 차선 변경).

        원래는 'lane2'가 하드코딩돼 있었다. 이 한 줄을 파라미터로 빼는 것만으로
        차선 변경이 된다 — 회피 노드가 조향을 직접 만들 필요가 없고, 기존
        경로계획이 알아서 부드러운 곡선을 그려준다.
        """
        if msg.data and msg.data != self.target_lane:
            self.get_logger().info(f'따라갈 차선 변경: {self.target_lane} -> {msg.data}')
            self.target_lane = msg.data

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        if len(detection_msg.detections) == 0:
            return

        # 따라갈 차선이 이번 프레임에 안 잡혔으면 **그냥 넘긴다.**
        #
        # 없는 차선으로 차선 중앙을 구하면 빈 화면에서 계산하게 되고, 엉뚱한 목표점이
        # 나와서 핸들이 튄다. 특히 회피 중(옆 차선을 따라가는 중)에 터지면 위험하다.
        # 실측(녹화 영상): lane2는 100% 잡히지만 **lane1은 73.8%뿐이다** — 즉 옆 차선은
        # 4프레임 중 1번꼴로 안 보인다. 흔한 일이므로 반드시 막아야 한다.
        #
        # 한 프레임 건너뛰어도 문제없다(30Hz). path_planner가 직전 값을 그대로 쓴다.
        if not any(d.class_name == self.target_lane for d in detection_msg.detections):
            self.get_logger().warn(f'{self.target_lane}이 안 보임 — 이 프레임은 건너뜀',
                                   throttle_duration_sec=1.0)
            return

        lane2_edge_image = CPFL.draw_edges(detection_msg, cls_name=self.target_lane, color=255)

        (h, w) = (lane2_edge_image.shape[0], lane2_edge_image.shape[1]) #(480, 640)
        dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)], [round(w * 0.7), h], [round(w * 0.3), h]]

        lane2_bird_image = CPFL.bird_convert(lane2_edge_image, srcmat=self.src_mat, dstmat=dst_mat)
        roi_image = CPFL.roi_rectangle_below(lane2_bird_image, cutting_idx=self.cutting_idx)

        if self.show_image:
            cv2.imshow('lane2_edge_image', lane2_edge_image)
            cv2.imshow('lane2_bird_img', lane2_bird_image)
            cv2.imshow('roi_img', roi_image)
            cv2.waitKey(1)

        # roi_image를 uint8 형식으로 변환 (아래 gradient/center 계산이 이 형식을 쓴다)
        roi_image = cv2.convertScaleAbs(roi_image)  # 64FC1 -> uint8로 변환

        # ROI 이미지 발행은 **디버그용**이라 show_image일 때만 한다.
        # 큰 이미지를 DDS로 한 벌 더 흘리는 비용이 있다 (프레임 유실 이력 — RUNTIME_FIXES.md).
        if self.show_image:
            try:
                roi_image_msg = self.cv_bridge.cv2_to_imgmsg(roi_image, encoding="mono8")
                self.roi_image_publisher.publish(roi_image_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to convert and publish ROI image: {e}")
        
        grad = CPFL.dominant_gradient(roi_image, theta_limit=70)

        target_points = []
        for target_point_y in range(5, 155, 50):  # 예시로 5에서 155까지 50씩 증가
            target_point_x = CPFL.get_lane_center(roi_image, detection_height=target_point_y,
                                                detection_thickness=self.detection_thickness,
                                                road_gradient=grad, lane_width=self.lane_width)
            
            target_point = TargetPoint()
            target_point.target_x = round(target_point_x)
            target_point.target_y = round(target_point_y)
            target_points.append(target_point)

        lane = LaneInfo()
        lane.slope = grad
        lane.target_points = target_points

        self.publisher.publish(lane)


def main(args=None):
    rclpy.init(args=args)
    node = Yolov8InfoExtractor()
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
