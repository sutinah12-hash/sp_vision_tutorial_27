#!/usr/bin/env bash
# Run on Ubuntu 22.04: sudo bash scripts/install_dependencies.sh [http://host:port]
set -euo pipefail
test "$(id -u)" = 0 || { echo 'Run this script with sudo.'; exit 1; }
. /etc/os-release
test "$VERSION_ID" = '22.04' || { echo 'Ubuntu 22.04 is required.'; exit 1; }
apt_options=(-o DPkg::Lock::Timeout=300 -o Acquire::Retries=3)
if [[ -n "${1:-}" ]]; then
  export http_proxy="$1" https_proxy="$1"
  apt_options+=(-o "Acquire::http::Proxy=$1" -o "Acquire::https::Proxy=$1")
fi
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  source_deb=$(mktemp /tmp/nav_ros2_source.XXXXXX.deb)
  wget -q --timeout=45 --tries=3 -O "$source_deb" \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb
  while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    echo 'Waiting for the current system package operation...'
    sleep 5
  done
  dpkg -i "$source_deb"
fi
apt-get "${apt_options[@]}" update
# --no-remove protects installed system/desktop packages from removal.
apt-get "${apt_options[@]}" --no-remove install -y \
  ros-humble-ros-base python3-colcon-common-extensions python3-rosdep \
  ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs ros-humble-std-srvs ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 ros-humble-ament-cmake-gtest \
  qtbase5-dev libopencv-dev libyaml-cpp-dev libeigen3-dev \
  python3-pygame python3-numpy python3-pil python3-scipy python3-matplotlib \
  xdotool wmctrl
echo 'Navigation dependencies installed. Source /opt/ros/humble/setup.bash to use ROS2.'
