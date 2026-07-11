#!/usr/bin/env bash
# ROS2 작업 환경 설정 — 터미널 열 때마다 source 한다.
#
#   source ~/Desktop/Projects/SKKU_AutoDrive/scripts/setup_env.sh
#
# 매번 치기 싫으면 ~/.bashrc 맨 아래에 위 한 줄을 추가한다.
#
# 여기서 하는 일
#  1) ROS2 Jazzy + 우리 워크스페이스 환경 등록
#  2) Fast DDS가 **큰 소켓 버퍼를 요청**하도록 프로파일 지정
#     → 카메라 이미지(921KB)가 커널 UDP 기본 버퍼(208KB)에 안 들어가서
#       BEST_EFFORT 구독자(yolov8 등)가 프레임의 80%를 잃던 문제 해결
#       (실측: 6.6Hz → 33.4Hz, 유실 0)
#
# [주의] 커널 상한도 함께 열려 있어야 한다 (sudo, 1회):
#   echo -e 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216' \
#     | sudo tee /etc/sysctl.d/60-ros2-image.conf
#   sudo sysctl --system
#   ※ rmem_max는 '허용 상한'일 뿐이라, DDS가 요청까지 해야(위 2번) 실제로 커진다. 둘 다 필요.

_SKKU_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash

if [ -f "${_SKKU_ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${_SKKU_ROOT}/ros2_ws/install/setup.bash"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${_SKKU_ROOT}/ros2_ws/fastdds_bigbuf.xml"

# YOLO 가중치 위치 (yolov8_node가 여기부터 찾는다 — 없으면 레포 루트/CWD 순으로 탐색)
export SKKU_MODELS_DIR="${_SKKU_ROOT}/models"

echo "[SKKU_AutoDrive] ROS2 Jazzy + 워크스페이스 환경 등록 완료"
echo "  DDS 프로파일: $(basename "$FASTRTPS_DEFAULT_PROFILES_FILE") (큰 소켓 버퍼)"
_rmem=$(sysctl -n net.core.rmem_max 2>/dev/null)
if [ "${_rmem:-0}" -lt 8388608 ]; then
    echo "  [경고] net.core.rmem_max=${_rmem} — 이미지가 유실됩니다. 위 주석의 sysctl 설정을 하세요."
fi
