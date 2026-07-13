import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):

        if self.lidar_data is not None and self.lidar_data.data is True:
            # 라이다가 장애물을 감지한 경우
            self.steering_command = 0 
            self.left_speed_command = 0 
            self.right_speed_command = 0 

        elif (self.traffic_light_data is not None
              and self.traffic_light_data.data == 'Red'
              # detection_data도 확인해야 한다 — 원본은 traffic_light_data만 검사하고 아래에서
              # self.detection_data.detections를 바로 썼다. 신호등 메시지가 첫 감지 결과보다
              # 먼저 도착하면(시작 직후 흔함) None.detections → AttributeError로 노드가 죽는다.
              and self.detection_data is not None):
            # 빨간색 신호등을 감지한 경우
            for detection in self.detection_data.detections:
                if detection.class_name=='traffic_light':
                    x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
                    x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
                    y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
                    y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

                    if y_max < 150:
                        # 신호등 위치에 따른 정지명령 결정
                        self.steering_command = 0 
                        self.left_speed_command = 0 
                        self.right_speed_command = 0
        else:
            if self.path_data is None:
                # 경로 없음: 조향 중립 (target_slope 미정의이므로 convert 호출 안 함 → 기존 NameError 버그 수정)
                self.steering_command = 0
            else:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.steering_command = convert_steeringangle2command(52, target_slope)  # TODO: 52 = 최대조향 대응 slope 기준값, 카메라 각도/트랙 곡률로 재조정
            self.left_speed_command = 80   # TODO: 모터 사양(전압/기어비)에 맞게 실측 조정
            self.right_speed_command = 80  # TODO: 좌우 균형 실측 조정



        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
def convert_steeringangle2command(max_target_angle, target_angle):
   
    f = lambda x : 7/(max_target_angle**3)*(x**3) #64000
    ret_direction = round(f(target_angle))
 
    ret_direction = 7 if ret_direction >= 7 else ret_direction
    ret_direction = -7 if ret_direction <= -7 else ret_direction
    #print('angle_control_direction: ', ret_direction)
    return ret_direction