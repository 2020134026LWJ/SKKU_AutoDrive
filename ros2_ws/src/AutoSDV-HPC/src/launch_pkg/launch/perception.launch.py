# perception.launch.py
#
# 하드웨어 없이 "인지 파이프라인"만 켠다. (Phase 2 검증 / 코드 확인용)
#   레포에 들어있는 녹화 영상(data_source='video')을 카메라 대신 재생 →
#   YOLO → 차선 → 경로 → 판단 까지 흐르는지 본다.
#   시리얼(아두이노)·라이다(실장치)는 포함 안 함. → 아두이노/라이다 없어도 돈다.
#
# 실행:  ros2 launch launch_pkg perception.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 카메라 대신 녹화 영상 재생 → image_01 로 발행
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='image_publisher_node', output='screen',
            parameters=[{'data_source': 'video', 'pub_topic': 'image_01'}],  # 실카메라는 main.launch.py 참고
        ),
        # image_01 구독 → YOLO 검출 → detections 발행
        Node(
            package='camera_perception_pkg', executable='yolov8_node',
            name='yolov8_node', output='screen',
        ),
        # detections 구독 → 차선 중앙점 추출 → yolov8_lane_info 발행
        Node(
            package='camera_perception_pkg', executable='lane_info_extractor_node',
            name='lane_info_extractor_node', output='screen',
        ),
        # yolov8_lane_info 구독 → 경로(CubicSpline) → path_planning_result 발행
        Node(
            package='decision_making_pkg', executable='path_planner_node',
            name='path_planner_node', output='screen',
        ),
        # (선택) 신호등: image_01 구독 → 색 판정 → yolov8_traffic_light_info 발행
        Node(
            package='camera_perception_pkg', executable='traffic_light_detector_node',
            name='traffic_light_detector_node', output='screen',
        ),
        # 최종 판단: 경로/신호등(+라이다 없으면 무시) → topic_control_signal 발행
        #   ※ 여기까지만. serial_sender 는 아두이노 포트를 열어서 하드웨어 없으면 크래시 → 제외.
        Node(
            package='decision_making_pkg', executable='motion_planner_node',
            name='motion_planner_node', output='screen',
        ),
    ])
