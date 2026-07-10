# lidar_perception_func_lib
#
# 복원 이력 (2026-07-11):
#   원본은 소스 없이 cpython-310 .pyc로만 존재 → Python 3.12(Jazzy)에서 로드 불가.
#   pycdc 디컴파일 + 바이트코드(pycdas) 대조로 재구성.
#   - RPLidar / RPLidarException: 공개 라이브러리 `rplidar`(SkoltechRobotics, MIT)를 vendoring한 것이라
#     디컴파일 대신 라이브러리에서 재export (원 코드 lidar_publisher_node.py:14 주석과 동일 의도).
#     설치: pip install rplidar-roboticia
#   - detect_object / check_consecutive_detections: pycdc가 조용히 손상시킨 부분을
#     바이트코드 기준으로 정확히 복원.
from rplidar import RPLidar, RPLidarException  # noqa: F401  (LPFL.RPLidar / LPFL.RPLidarException 재export)


def rotate_lidar_data(msg, offset=0):
    offset = int(offset)
    if offset < 0 or offset >= 360:
        raise ValueError('offset must be between 0 and 359')
    msg.ranges = msg.ranges[offset:] + msg.ranges[:offset]
    msg.intensities = msg.intensities[offset:] + msg.intensities[:offset]
    return msg


def flip_lidar_data(msg, pivot_angle):
    pivot_angle = int(pivot_angle)
    if pivot_angle < 0 or pivot_angle >= 360:
        raise ValueError('pivot_angle must be between 0 and 359')
    length = len(msg.ranges)
    flipped_ranges = [0] * length
    flipped_intensities = [0] * length
    for i in range(length):
        new_angle = (2 * pivot_angle - i) % length
        flipped_ranges[new_angle] = msg.ranges[i]
        flipped_intensities[new_angle] = msg.intensities[i]
    msg.ranges = flipped_ranges
    msg.intensities = flipped_intensities
    return msg


def detect_object(ranges, start_angle, end_angle, range_min, range_max):
    num_readings = len(ranges)
    if start_angle > end_angle:
        end_angle += num_readings
    for i in range(start_angle, end_angle + 1):
        index = i % num_readings
        if range_min <= ranges[index] <= range_max:
            return True
    return False


class StabilityDetector:

    def __init__(self, consec_count):
        self.consec_count = consec_count
        self.detection_history = []
        self.current_state = False

    def check_consecutive_detections(self, detected):
        ''' 감지가 연속으로 일어나는지 확인하는 함수

        Parameters:
        detected (bool): 현재 감지된 여부

        Returns:
        bool: 감지 상태가 변했는지 여부
        '''
        self.detection_history.append(detected)
        if len(self.detection_history) > self.consec_count:
            self.detection_history.pop(0)
        if self.current_state and self.detection_history.count(False) >= self.consec_count:
            self.current_state = False
            return self.current_state
        if not self.current_state and self.detection_history.count(True) >= self.consec_count:
            self.current_state = True
        return self.current_state
