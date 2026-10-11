#!/usr/bin/env bash
set -Eeuo pipefail
ARM1_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARM1_APP="$ARM1_WORKSPACE/integration/arm1"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
unset PYTHONHOME PYTHONPATH
export HF_HUB_OFFLINE=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
export ROS_LOG_DIR="$ARM1_APP/logs/ros"
mkdir -p "$ROS_LOG_DIR"
ARM1_PYTHON="$ARM1_WORKSPACE/arm3-current/.venv/bin/python"
if [[ ! -x "$ARM1_PYTHON" ]]; then ARM1_PYTHON=/usr/bin/python3; fi
# Local workaround for the still-loaded pre-update NVIDIA kernel, PC only.
# A normal reboot to another driver version bypasses this block automatically.
if [[ "$mode" == serve || "$mode" == check-models ]]; then
  ARM1_CUDA_LIB="$ARM1_WORKSPACE/runtime/nvidia-580.173.02/extracted/usr/lib/x86_64-linux-gnu"
  if [[ -r /sys/module/nvidia/version && -f "$ARM1_CUDA_LIB/libcuda.so.580.173.02" ]]; then
    if [[ "$(cat /sys/module/nvidia/version)" == "580.173.02" ]]; then
      export LD_LIBRARY_PATH="$ARM1_CUDA_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
      echo '관제 CUDA: 현재 커널에 맞는 프로젝트 내 580.173.02 라이브러리 사용'
    fi
  fi
fi
case "$mode" in
  setup) exec bash "$ARM1_APP/setup.sh" "$@" ;;
  check-models) exec "$ARM1_WORKSPACE/arm3-current/.venv/bin/python" "$ARM1_APP/check_models.py" "$@" ;;
  test) exec "$ARM1_PYTHON" -m unittest discover -s "$ARM1_APP/tests" -p 'test_*.py' -v ;;
  test-ros)
    source "$ARM1_APP/ros-env.sh"
    exec /usr/bin/python3 "$ARM1_APP/tests/run_ros_suite.py" "$@" ;;
  serve|send|check-connection)
    source "$ARM1_APP/ros-env.sh"
    if [[ "$mode" == serve ]]; then
      exec "$ARM1_PYTHON" -u "$ARM1_APP/server.py" "$@"
    elif [[ "$mode" == check-connection ]]; then
      exec /usr/bin/python3 -u "$ARM1_APP/client.py" --check-connection "$@"
    else
      exec /usr/bin/python3 -u "$ARM1_APP/client.py" "$@"
    fi ;;
  *) echo 'Usage: bash arm1-control.sh {setup|check-models|test|test-ros|serve|send|check-connection} [options]'
     echo 'serve/send default to localhost MOCK domain 141; --hardware selects real robot domain 40.' ;;
esac
