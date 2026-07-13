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
SUB_PARKING_ACTIVE_TOPIC_NAME = "parking_active"
SUB_PARKING_COMMAND_TOPIC_NAME = "parking_command"
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
        self.sub_parking_active_topic = self.declare_parameter('sub_parking_active_topic', SUB_PARKING_ACTIVE_TOPIC_NAME).value
        self.sub_parking_command_topic = self.declare_parameter('sub_parking_command_topic', SUB_PARKING_COMMAND_TOPIC_NAME).value
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
        self.parking_active = False
        self.parking_command = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.parking_active_sub = self.create_subscription(Bool, self.sub_parking_active_topic, self.parking_active_callback, self.qos_profile)
        self.parking_command_sub = self.create_subscription(MotionCommand, self.sub_parking_command_topic, self.parking_command_callback, self.qos_profile)

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

    def parking_active_callback(self, msg: Bool):
        self.parking_active = msg.data

    def parking_command_callback(self, msg: MotionCommand):
        self.parking_command = msg

    def timer_callback(self):

        # 주차 중에는 parking_controller_node에 제어를 넘긴다.
        #
        # [주의] 이 분기는 반드시 라이다 분기보다 먼저 와야 한다. 주차는 뒤쪽 장애물
        # (벽/주차선)에 의도적으로 다가가는 동작이라, 아래 "장애물 감지 → 정지"에
        # 먼저 걸리면 주차칸 입구에서 그대로 멈춰버린다.
        if self.parking_active and self.parking_command is not None:
            self.steering_command = self.parking_command.steering
            self.left_speed_command = self.parking_command.left_speed
            self.right_speed_command = self.parking_command.right_speed

        elif self.lidar_data is not None and self.lidar_data.data is True:
            # 라이다가 장애물을 감지한 경우
            self.steering_command = 0 
            self.left_speed_command = 0 
            self.right_speed_command = 0 

        elif (self.traffic_light_data is not None
              and self.traffic_light_data.data == 'Red'):
            # 빨간불 → 정지.
            #
            # 원본은 여기서 YOLO의 `class_name=='traffic_light'` bbox를 뒤져 y_max<150 인지
            # 봤다. 그런데 **best_urp.pt 에는 traffic_light 클래스가 없다** → for문이 항상
            # 빈 채로 끝나서 (1) 진짜 빨간불이어도 절대 안 멈추고, (2) 그런데 elif에는
            # 들어갔으므로 아래 else(정상 주행)를 건너뛰어 **직전 조향/속도 명령이 그대로
            # 계속 발행**됐다 — 차선 추종이 얼어붙는다. 검출부만 HSV(대안 A)로 바꾸고
            # 판단부는 YOLO에 남겨둔 탓이다.
            #
            # 이제 신호등 검출기가 색을 직접 확정해 주므로(3프레임 연속 투표) 그 값만 믿는다.
            # '얼마나 가까운가'는 검출기의 min_area/max_area가 대신한다 — 멀어서 작게
            # 보이는 신호등은 애초에 검출되지 않는다.
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