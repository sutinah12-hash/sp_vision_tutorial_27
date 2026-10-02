#!/usr/bin/env bash
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "Build first: cd $WS_DIR && colcon build --base-paths src --symlink-install"
  exit 1
fi
source "$WS_DIR/install/setup.bash"
export QT_QPA_PLATFORM=xcb
exec ros2 launch sp_nav_bringup project.launch.py "$@"
