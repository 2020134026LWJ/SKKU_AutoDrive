import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DDSImageListener(Node):
    def __init__(self):
        super().__init__('dds_image_publisher')
        self.bridge = CvBridge()

        # 'image_number'라는 이름의 파라미터를 선언하고 기본값을 'image_topic'으로 설정합니다.
        self.declare_parameter('image', 'image_01')
        # 파라미터 값을 가져옵니다.
        topic_name = self.get_parameter('image').get_parameter_value().string_value
        
        # 가져온 파라미터 값(topic_name)을 사용하여 퍼블리셔를 생성합니다.
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.get_logger().info(f'Publishing to topic: {topic_name}')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # DDS와 유사한 토픽(DDS 입력을 시뮬레이션하는 ROS 2 토픽)을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            topic_name+'_raw',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('DDS Image Listener Node has started.')

    def listener_callback(self, msg):
        try:
            # ROS 2 이미지 메시지에서 직접 JPEG 데이터를 디코딩합니다.
            jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode JPEG image.')
                return

            # 이미지를 처리합니다 (예: 화면에 표시하거나 수정).
            cv2.imshow("DDS Image Viewer", cv_image)
            cv2.waitKey(1)

            # 선택적으로 디코딩된 이미지를 다시 발행합니다.
            decoded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_.publish(decoded_msg)
            # self.get_logger().info(f'Image received and republished: {cv_image.shape[1]}x{cv_image.shape[0]}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DDSImageListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DDS Image Listener Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
