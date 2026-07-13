import os

from setuptools import find_packages, setup

package_name = 'camera_perception_pkg'

# 테스트용 주행 영상 — install 트리엔 .py만 복사되므로(데이터는 안 따라옴) share/에 설치한다.
# 노드는 get_package_share_directory()로 찾는다 → 실행 위치·빌드 방식과 무관.
# (sample_dataset 이미지 1259장은 무겁고 'image' 소스에서만 쓰므로 설치하지 않는다)
_VIDEO = os.path.join(package_name, 'lib', 'Collected_Datasets', 'driving_simulation.mp4')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'Collected_Datasets'),
            [_VIDEO] if os.path.exists(_VIDEO) else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhk',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = camera_perception_pkg.image_publisher_node:main',
            'ethernet_image_publisher_node = camera_perception_pkg.ethernet_image_publisher_node:main',
            'yolov8_node = camera_perception_pkg.yolov8_node:main',
            'traffic_light_detector_node = camera_perception_pkg.traffic_light_detector_node:main',
            'lane_info_extractor_node = camera_perception_pkg.lane_info_extractor_node:main',
            'rear_park_detector_node = camera_perception_pkg.rear_park_detector_node:main',
        ],
    },
)
