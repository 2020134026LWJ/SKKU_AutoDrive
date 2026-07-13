# bench.launch.py
#
# 하드웨어 없이 **전체 파이프라인을 한 번에** 띄운다.
#
#   ros2 launch launch_pkg bench.launch.py
#
# main.launch.py 와 같은 노드 구성이되, 차가 없어도 뜨도록:
#   - 카메라 2대  → 녹화 영상 (data_source:=video). 전방/후방 둘 다 같은 영상.
#   - 라이다      → 제외 (장치가 없으면 노드가 죽는다)
#   - 시리얼 송신 → 제외 (import 시 아두이노 포트를 열어서 없으면 크래시)
#
# 이걸 왜 만드나:
#   노드를 여러 개 추가했는데 **다 같이 띄워본 적이 없으면** 이름 오타 / 파라미터 불일치 /
#   토픽 안 맞음 같은 게 **현장에서 처음** 터진다. 그런 건 여기서 10분에 잡는다.
#   (실측값이 틀린 건 여기서 못 잡는다 — 그건 차가 있어야 한다)
#
# 확인할 것:
#   ros2 node list                    노드가 다 살아있나
#   ros2 topic echo /target_lane      회피 노드가 차선을 정하고 있나
#   ros2 topic echo /topic_control_signal   조향/속도가 나오나
#   ros2 topic hz /image_01           프레임이 유실되지 않나 (카메라 2대라 트래픽 2배)
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _find_calibration_yaml():
    env = os.environ.get('SKKU_CALIB')
    if env and os.path.isfile(env):
        return env
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        cand = os.path.join(d, 'config', 'calibration.yaml')
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return ''


def generate_launch_description():
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=_find_calibration_yaml()),

        # ---------- 카메라 (녹화 영상으로 대체) ----------
        Node(package='camera_perception_pkg', executable='image_publisher_node',
             name='image_publisher_front', output='screen',
             parameters=[{'data_source': 'video', 'pub_topic': 'image_01'}]),
        Node(package='camera_perception_pkg', executable='image_publisher_node',
             name='image_publisher_rear', output='screen',
             parameters=[{'data_source': 'video', 'pub_topic': 'image_02'}]),

        # ---------- 인지 ----------
        Node(package='camera_perception_pkg', executable='yolov8_node',
             name='yolov8_node', output='screen'),
        Node(package='camera_perception_pkg', executable='lane_info_extractor_node',
             name='lane_info_extractor_node', output='screen', parameters=[params]),
        Node(package='camera_perception_pkg', executable='traffic_light_detector_node',
             name='traffic_light_detector_node', output='screen', parameters=[params]),
        Node(package='camera_perception_pkg', executable='rear_park_detector_node',
             name='rear_park_detector_node', output='screen', parameters=[params]),

        # ---------- 판단 ----------
        Node(package='decision_making_pkg', executable='path_planner_node',
             name='path_planner_node', output='screen', parameters=[params]),
        Node(package='decision_making_pkg', executable='obstacle_avoider_node',
             name='obstacle_avoider_node', output='screen', parameters=[params]),
        Node(package='decision_making_pkg', executable='parking_controller_node',
             name='parking_controller_node', output='screen', parameters=[params]),
        Node(package='decision_making_pkg', executable='motion_planner_node',
             name='motion_planner_node', output='screen', parameters=[params]),

        # 라이다 / 시리얼은 장치가 필요하므로 제외 (main.launch.py 에는 있다)
    ])
