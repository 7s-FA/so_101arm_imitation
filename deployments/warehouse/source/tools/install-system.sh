#!/usr/bin/env bash
set -Eeuo pipefail
[[ $EUID -ne 0 ]] || { echo '일반 사용자로 실행하세요(sudo는 필요한 명령에만 사용).' >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && "$(uname -m)" == aarch64 ]] || exit 2
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev python3-tk python3-pip build-essential cmake \
  curl ca-certificates software-properties-common libopenblas-dev libusb-1.0-0-dev \
  libgl1 libglib2.0-0 ffmpeg fontconfig fonts-noto-cjk psmisc udev v4l-utils usbutils
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  # Official ROS apt-source bootstrap, only when Jazzy is absent.
  sudo add-apt-repository -y universe
  ARM1_INSTALL_TMP="$(mktemp -d)"
  trap 'rm -rf -- "$ARM1_INSTALL_TMP"' EXIT
  curl --fail --location --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$ARM1_INSTALL_TMP/release.json"
  ARM1_ROS_APT_VERSION="$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$ARM1_INSTALL_TMP/release.json")"
  [[ "$ARM1_ROS_APT_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Unexpected ROS apt-source release tag'; exit 2; }
  curl --fail --location --retry 3 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ARM1_ROS_APT_VERSION}/ros2-apt-source_${ARM1_ROS_APT_VERSION}.noble_all.deb" -o "$ARM1_INSTALL_TMP/ros2-apt-source.deb"
  sudo dpkg -i "$ARM1_INSTALL_TMP/ros2-apt-source.deb"
  sudo apt-get update
fi
sudo apt-get install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-rosidl-default-generators
sudo usermod -aG dialout,video "$(id -un)"
printf '%s\n' 'SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b07", MODE="0660", GROUP="video", TAG+="uaccess"' |
  sudo tee /etc/udev/rules.d/99-arm1-d435.rules >/dev/null
sudo udevadm control --reload-rules
echo 'USB 권한 적용: 로그아웃/로그인 후 장치를 다시 연결하세요.'
