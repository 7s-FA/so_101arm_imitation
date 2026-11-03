#!/usr/bin/env bash
# Runs on Jetson, never on the source training PC.
set -Eeuo pipefail
ARM1_PACKAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ARM1_PACKAGE"
case "${1:-}" in
  -h|--help)
    echo 'Usage: bash install-jetson.sh [--ik-only] [--system] [--python /existing/venv/bin/python] [--torch-wheel PATH_OR_URL --torchvision-wheel PATH_OR_URL] [--realsense-wheel PATH_OR_URL]'
    echo '--ik-only: 10/08 패키지 환경이 동작할 때, 동봉한 mujoco/scipy(ARM64)만 오프라인으로 추가합니다.'
    echo 'Ubuntu 24.04 ARM64 / Python 3.12 / ROS Jazzy. Keeps compatible installed CUDA torch; never guesses JetPack wheels.'
    exit 0 ;;
esac
[[ "$(uname -m)" == aarch64 ]] || { echo 'Jetson ARM64에서 실행하세요. 이 PC에는 설치하지 않습니다.' >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || {
  echo '이 배포본은 Ubuntu 24.04 + Python 3.12 + ROS Jazzy용입니다. 다른 JetPack/OS는 README의 Codex 절차를 따르세요. OS를 자동 변경하지 않습니다.' >&2; exit 2;
}
unset PYTHONHOME PYTHONPATH
export PYTHONNOUSERSITE=1
args=()
for arg in "$@"; do
  if [[ "$arg" == --system ]]; then bash tools/install-system.sh; else args+=("$arg"); fi
done
/usr/bin/python3 tools/install_runtime.py "${args[@]}"
bash integration/arm1/setup.sh
bash jetson-run.sh doctor --require-cuda
echo '설치 점검 완료. 다음: bash jetson-run.sh ik-test  (IK 보정 계획 생성 시험, 장치 미사용)'
echo '그 다음: bash jetson-run.sh check-models --device cuda'
echo '그 다음: bash jetson-run.sh benchmark --device cuda (장치 미사용)'
