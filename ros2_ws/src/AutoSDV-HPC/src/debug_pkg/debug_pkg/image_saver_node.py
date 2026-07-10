import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces_pkg.msg import MotionCommand
from cv_bridge import CvBridge
import os
import time
import message_filters
import cv2

class MultiCamImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        self.bridge = CvBridge()
        base_save_dir = os.path.expanduser('~/parking')
        
        cam_folders = ['cam1', 'cam2', 'cam3', 'cam4'] 
        
        self.save_dirs = []
        for folder in cam_folders:
            path = os.path.join(base_save_dir, folder)
            os.makedirs(path, exist_ok=True)
            self.save_dirs.append(path)

        self.steering = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0

        self.control_sub = self.create_subscription(
            MotionCommand, 
            '/topic_control_signal', 
            self.data_callback, 
            10
        )

        self.image_sub_1 = message_filters.Subscriber(self, Image, '/image_01')
        self.image_sub_2 = message_filters.Subscriber(self, Image, '/image_02')
        self.image_sub_3 = message_filters.Subscriber(self, Image, '/image_03')
        self.image_sub_4 = message_filters.Subscriber(self, Image, '/image_04')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub_1, self.image_sub_2, self.image_sub_3, self.image_sub_4],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

    def data_callback(self, msg: MotionCommand):
        self.steering = msg.steering
        self.left_speed = msg.left_speed
        self.right_speed = msg.right_speed

    def image_callback(self, img1, img2, img3, img4):
        try:
            # --- [수정] 모터 속도가 0일 경우 저장을 건너뛰는 로직 ---
            # 좌우 모터 속도가 모두 0이면 함수를 즉시 종료하여 아래 코드를 실행하지 않음
            if self.left_speed == 0.0 and self.right_speed == 0.0:
                self.get_logger().info('Skipping image save: motor speed is zero.') # 필요 시 주석 해제하여 로그 확인
                return
            # --- [수정 끝] ---

            cv_img1 = self.bridge.imgmsg_to_cv2(img1)
            cv_img2 = self.bridge.imgmsg_to_cv2(img2)
            cv_img3 = self.bridge.imgmsg_to_cv2(img3)
            cv_img4 = self.bridge.imgmsg_to_cv2(img4)

            timestamp = int(time.time() * 1000)
            base_name = f'image_{timestamp}_s{self.steering:.2f}_l{self.left_speed:.2f}_r{self.right_speed:.2f}'

            # 각각의 카메라 이미지 저장
            cv2.imwrite(os.path.join(self.save_dirs[0], base_name + '_cam1.png'), cv_img1)
            cv2.imwrite(os.path.join(self.save_dirs[1], base_name + '_cam2.png'), cv_img2)
            cv2.imwrite(os.path.join(self.save_dirs[2], base_name + '_cam3.png'), cv_img3)
            cv2.imwrite(os.path.join(self.save_dirs[3], base_name + '_cam4.png'), cv_img4)

            self.get_logger().info(f'이미지 저장 완료: {base_name}.png')

        except Exception as e:
            self.get_logger().error(f'이미지 저장 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()