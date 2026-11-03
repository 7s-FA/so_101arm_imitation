#!/usr/bin/env bash
# Source only inside a launcher subprocess; never changes ~/.bashrc.
ARM1_ROS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
if ! /usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; get_package_prefix("host_pkg")' >/dev/null 2>&1; then
  if [[ ! -f "$ARM1_ROS_DIR/ros_ws/install/local_setup.bash" ]]; then
    echo 'Arm interface missing. Run: bash arm1-control.sh setup' >&2
    exit 2
  fi
  source "$ARM1_ROS_DIR/ros_ws/install/local_setup.bash"
fi
# A nested launcher clears PYTHONPATH but may inherit AMENT_PREFIX_PATH. Restore
# the selected existing package's own hooks, not another same-named overlay.
ARM1_HOST_PREFIX="$(/usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("host_pkg"))')"
source "$ARM1_HOST_PREFIX/share/host_pkg/local_setup.bash"
set -u
/usr/bin/python3 "$ARM1_ROS_DIR/interface_check.py"
