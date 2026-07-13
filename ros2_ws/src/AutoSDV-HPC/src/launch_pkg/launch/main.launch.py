# main.launch.py
#
# 실주행 전체 파이프라인. (하드웨어 다 연결된 상태)
#   전방/후방 카메라 + 라이다 + 인지/판단 + 아두이노 시리얼 송신.
#
# 실행:  ros2 launch launch_pkg main.launch.py
#
# ※ 원본은 ethernet_image_publisher 5개(다른 하드웨어 구성)만 켜져 있었음 → 우리 구성으로 교체.
# ※ 하드웨어별 실측값은 docs/CALIBRATION.md 참고. 장치 번호/포트는 아래 TODO.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---------- 카메라 ----------
        # 전방 카메라 → image_01
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='image_publisher_front', output='screen',
            parameters=[{'data_source': 'camera', 'cam_num': 0, 'pub_topic': 'image_01'}],  # TODO: cam_num = ls /dev/video* 로 확인
        ),
        # 후방 카메라 → image_02  (활용 노드는 추후 작성)
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='image_publisher_rear', output='screen',
            parameters=[{'data_source': 'camera', 'cam_num': 1, 'pub_topic': 'image_02'}],  # TODO: cam_num 확인
        ),

        # ---------- 카메라 인지 ----------
        Node(package='camera_perception_pkg', executable='yolov8_node',
             name='yolov8_node', output='screen'),
        Node(package='camera_perception_pkg', executable='lane_info_extractor_node',
             name='lane_info_extractor_node', output='screen'),
        Node(package='camera_perception_pkg', executable='traffic_light_detector_node',
             name='traffic_light_detector_node', output='screen'),

        # ---------- 라이다 (후방 장애물) ----------
        Node(package='lidar_perception_pkg', executable='lidar_publisher_node',
             name='lidar_publisher_node', output='screen'),   # TODO: LIDAR_PORT = ls /dev/ttyUSB*
        Node(package='lidar_perception_pkg', executable='lidar_processor_node',
             name='lidar_processor_node', output='screen'),   # TODO: 장착방향 offset 보정
        Node(package='lidar_perception_pkg', executable='lidar_obstacle_detector_node',
             name='lidar_obstacle_detector_node', output='screen'),

        # ---------- 경로/판단 ----------
        Node(package='decision_making_pkg', executable='path_planner_node',
             name='path_planner_node', output='screen'),
        # 주차 FSM. parking_trigger를 받으면 parking_active를 올리고, motion_planner가 양보한다.
        #   ※ duration 값들은 개루프라 실측 필수 → docs/CALIBRATION.md "주차" 절
        Node(package='decision_making_pkg', executable='parking_controller_node',
             name='parking_controller_node', output='screen'),
        Node(package='decision_making_pkg', executable='motion_planner_node',
             name='motion_planner_node', output='screen'),

        # ---------- 아두이노 시리얼 송신 ----------
        #   ※ 이 노드는 import 시 아두이노 포트를 연다 → 아두이노 연결 안 됐으면 크래시.
        Node(package='serial_communication_pkg', executable='serial_sender_node',
             name='serial_sender_node', output='screen'),     # TODO: PORT = ls /dev/ttyACM*
    ])
