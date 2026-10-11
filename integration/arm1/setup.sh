#!/usr/bin/env bash
set -Eeuo pipefail
ARM1_SETUP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
if /usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; get_package_prefix("host_pkg")' >/dev/null 2>&1; then
  source "$ARM1_SETUP_DIR/ros-env.sh"
  echo 'Using existing compatible host_pkg; no package replaced.'
  exit 0
fi
cd "$ARM1_SETUP_DIR/ros_ws"
colcon build --packages-select host_pkg --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
set +u
source install/local_setup.bash
set -u
/usr/bin/python3 "$ARM1_SETUP_DIR/interface_check.py"
