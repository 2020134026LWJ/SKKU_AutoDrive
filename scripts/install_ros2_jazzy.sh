#!/usr/bin/env bash
# ROS2 Jazzy 설치 (Ubuntu 24.04) — sudo 필요
# 실행: bash scripts/install_ros2_jazzy.sh   (또는 Claude 프롬프트에 ! bash scripts/install_ros2_jazzy.sh)
set -euo pipefail

echo "=== 1. 로케일 ==="
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "=== 2. ROS2 apt 저장소 등록 ==="
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

echo "=== 3. ROS2 Jazzy + 우리 패키지 의존성 ==="
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-message-filters \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions

echo ""
echo "=== 완료. 아래를 ~/.bashrc 에 추가하거나 매 세션 source 하세요 ==="
echo "  source /opt/ros/jazzy/setup.bash"
echo ""
echo "설치 확인:  source /opt/ros/jazzy/setup.bash && ros2 --version"
