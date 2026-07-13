# main.launch.py
#
# 실주행 전체 파이프라인. (하드웨어 다 연결된 상태)
#   전방/후방 카메라 + 라이다 + 인지/판단 + 아두이노 시리얼 송신.
#
# 실행:  ros2 launch launch_pkg main.launch.py
#
# 실측값은 전부 **config/calibration.yaml** 에서 읽는다.
#   차에서 잰 숫자는 그 파일만 고치면 되고, 코드도 이 launch도 안 건드려도 된다.
#   (측정 절차: docs/CALIBRATION.md)
#
# [주의] yaml의 노드 이름과 여기 `name=` 이 **정확히 같아야** 값이 먹는다.
#        다르면 조용히 무시되고 기본값이 쓰인다. 확인:
#           ros2 param get /parking_controller_node speed_to_mps
#
# ※ 원본은 ethernet_image_publisher 5개(다른 하드웨어 구성)만 켜져 있었음 → 우리 구성으로 교체.
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _find_calibration_yaml():
    """레포의 config/calibration.yaml 을 찾는다.

    실행 시 __file__ 은 install 트리를 가리키는데, 그 install 트리도 레포 안
    (ros2_ws/install/...)에 있으므로 위로 거슬러 올라가면 레포 루트가 나온다.
    (모델 경로 찾기 `_resolve_model()` 과 같은 방식 — RUNTIME_FIXES.md 4번)

    env SKKU_CALIB 로 직접 지정할 수도 있다.
    """
    env = os.environ.get('SKKU_CALIB')
    if env and os.path.isfile(env):
        return env

    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):                      # 레포 루트까지 거슬러 올라간다
        cand = os.path.join(d, 'config', 'calibration.yaml')
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return ''                                # 못 찾으면 빈 값 → 기본값으로 동작


def generate_launch_description():
    # 실측값은 전부 config/calibration.yaml 에서 읽는다.
    # 다른 파일을 쓰려면:  ros2 launch launch_pkg main.launch.py params:=/경로/다른.yaml
    default_params = _find_calibration_yaml()
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params,
                              description='실측값 yaml (config/calibration.yaml)'),

        # ---------- 카메라 ----------
        # 전방 카메라 → image_01
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='image_publisher_front', output='screen',
            parameters=[{'data_source': 'camera', 'cam_num': 0, 'pub_topic': 'image_01'}],  # TODO: cam_num = ls /dev/video* 로 확인
        ),
        # 후방 카메라 → image_02  (주차의 눈: rear_park_detector_node가 본다)
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='image_publisher_rear', output='screen',
            parameters=[{'data_source': 'camera', 'cam_num': 1, 'pub_topic': 'image_02'}],  # TODO: cam_num 확인
        ),

        # ---------- 카메라 인지 ----------
        Node(package='camera_perception_pkg', executable='yolov8_node',
             name='yolov8_node', output='screen'),
        Node(package='camera_perception_pkg', executable='lane_info_extractor_node',
             name='lane_info_extractor_node', output='screen',
             parameters=[params]),
        Node(package='camera_perception_pkg', executable='traffic_light_detector_node',
             name='traffic_light_detector_node', output='screen',
             parameters=[params]),
        # 후방 카메라 → 뒤 경계선까지 거리 (주차 FSM이 이걸로 멈춘다)
        Node(package='camera_perception_pkg', executable='rear_park_detector_node',
             name='rear_park_detector_node', output='screen',
             parameters=[params]),

        # ---------- 라이다 (후방 장애물) ----------
        # [주의] 라이다가 장애물을 잡으면 motion_planner가 차를 **세운다.**
        #   미션 1의 장애물은 앞 카메라로 보고 **피하는** 것이므로, 라이다가 같은 것을
        #   보고 정지시키면 회피가 무력화된다. 감지 각도를 뒤쪽으로 잡아둘 것.
        Node(package='lidar_perception_pkg', executable='lidar_publisher_node',
             name='lidar_publisher_node', output='screen'),   # TODO: LIDAR_PORT = ls /dev/ttyUSB*
        Node(package='lidar_perception_pkg', executable='lidar_processor_node',
             name='lidar_processor_node', output='screen'),   # TODO: 장착방향 offset 보정
        Node(package='lidar_perception_pkg', executable='lidar_obstacle_detector_node',
             name='lidar_obstacle_detector_node', output='screen'),

        # ---------- 경로/판단 ----------
        Node(package='decision_making_pkg', executable='path_planner_node',
             name='path_planner_node', output='screen'),
        # 미션 1: 장애물 회피 = 차선 변경. 조향을 직접 만들지 않고 target_lane만 바꾼다.
        Node(package='decision_making_pkg', executable='obstacle_avoider_node',
             name='obstacle_avoider_node', output='screen',
             parameters=[params]),
        # 미션 3: 주차 FSM. parking_trigger를 받으면 parking_active를 올리고 motion_planner가 양보.
        Node(package='decision_making_pkg', executable='parking_controller_node',
             name='parking_controller_node', output='screen',
             parameters=[params]),
        # 제어 명령을 내는 입은 여기 하나뿐이다 (주차 중엔 주차 노드에게 양보).
        Node(package='decision_making_pkg', executable='motion_planner_node',
             name='motion_planner_node', output='screen',
             parameters=[params]),

        # ---------- 아두이노 시리얼 송신 ----------
        #   ※ 이 노드는 import 시 아두이노 포트를 연다 → 아두이노 연결 안 됐으면 크래시.
        Node(package='serial_communication_pkg', executable='serial_sender_node',
             name='serial_sender_node', output='screen'),     # TODO: PORT = ls /dev/ttyACM*
    ])
