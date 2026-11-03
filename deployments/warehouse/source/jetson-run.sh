#!/usr/bin/env bash
set -Eeuo pipefail
ARM1_PACKAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ARM1_PACKAGE"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
unset PYTHONPATH PYTHONHOME ARM3_PROJECT
export PYTHONNOUSERSITE=1 HF_HUB_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
export ROS_LOG_DIR="$ARM1_PACKAGE/integration/arm1/logs/ros"
if [[ "$mode" == verify ]]; then exec python3 tools/verify_package.py "$@"; fi
if [[ -z "${ARM1_PYTHON:-}" && -f .jetson-python ]]; then IFS= read -r ARM1_PYTHON < .jetson-python; fi
ARM1_PYTHON="${ARM1_PYTHON:-$ARM1_PACKAGE/.venv/bin/python}"
if [[ ! -x "$ARM1_PYTHON" ]]; then
  echo 'Python 환경이 없습니다. bash install-jetson.sh 실행 또는 ARM1_PYTHON 설정이 필요합니다.' >&2
  exit 2
fi
case "$mode" in
  A|B)
    # One process owns the physical hardware, including read-only checks.
    exec 9>"$ARM1_PACKAGE/.hardware-session.lock"
    flock -n 9 || { echo '이 패키지의 수동 실행 또는 관제 서버가 이미 실행 중입니다.' >&2; exit 2; }
    exec "$ARM1_PYTHON" -u tools/manual.py "$mode" "$@" ;;
  doctor) exec "$ARM1_PYTHON" -u tools/doctor.py "$@" ;;
  check-models) exec "$ARM1_PYTHON" -u integration/arm1/check_models.py "$@" ;;
  benchmark) exec "$ARM1_PYTHON" -u tools/benchmark.py "$@" ;;
  ik-test) exec "$ARM1_PYTHON" -u tools/ik_test.py "$@" ;;
  server|mock-server|test-ros)
    source integration/arm1/ros-env.sh
    if [[ "$mode" == server ]]; then
      exec 9>"$ARM1_PACKAGE/.hardware-session.lock"
      flock -n 9 || { echo '수동 실행 또는 관제 서버가 이미 실행 중입니다.' >&2; exit 2; }
      exec "$ARM1_PYTHON" -u integration/arm1/server.py --hardware "$@"
    elif [[ "$mode" == mock-server ]]; then
      exec "$ARM1_PYTHON" -u integration/arm1/server.py "$@"
    else
      exec /usr/bin/python3 integration/arm1/tests/run_ros_suite.py "$@"
    fi ;;
  test) exec "$ARM1_PYTHON" -m unittest discover -s integration/arm1/tests -p 'test_*.py' -v ;;
  *) echo 'Usage: bash jetson-run.sh {verify|doctor|ik-test|check-models|benchmark|A|B|server|mock-server|test|test-ros} [options]' ;;
esac
