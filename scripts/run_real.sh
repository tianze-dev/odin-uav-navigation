#!/usr/bin/env bash
set -eo pipefail
export SCUT_ROS_DOMAIN_ID="${SCUT_ROS_DOMAIN_ID:-0}"
export SCUT_PROFILE=real
source "$(dirname "$0")/env.sh"
# 设备驱动独立运行，不加载外部工作空间。
source "$SCUT_INSTALL/local_setup.bash"
# MVS 的 libusb 可能遮蔽 PCL 所需系统版本，仅调整本次导航进程。
scut_system_libusb="/usr/lib/$(uname -m)-linux-gnu/libusb-1.0.so.0"
if [[ -f "$scut_system_libusb" ]]; then
  export LD_PRELOAD="$scut_system_libusb${LD_PRELOAD:+:$LD_PRELOAD}"
fi
unset scut_system_libusb
exec ros2 launch uav_bringup real.launch.py root:="$SCUT_ROOT" "$@"
