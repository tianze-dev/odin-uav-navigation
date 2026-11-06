#!/usr/bin/env bash
# source 使用；真实驱动独立运行，本工程只需要标准 ROS 消息。
SCUT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SCUT_PROFILE="${SCUT_PROFILE:-real}"
case "$SCUT_PROFILE" in real) scut_default_ros=humble; scut_default_domain=0 ;; sim) scut_default_ros=jazzy; scut_default_domain=73 ;; *) echo 'SCUT_PROFILE must be real or sim' >&2; return 2 ;; esac
export SCUT_ROS_DISTRO="${SCUT_ROS_DISTRO:-$scut_default_ros}"
case "$SCUT_ROS_DISTRO" in humble|jazzy) ;; *) echo 'Use ROS 2 Humble or Jazzy' >&2; return 2 ;; esac
[[ -f "/opt/ros/$SCUT_ROS_DISTRO/setup.bash" ]] || { echo "Missing /opt/ros/$SCUT_ROS_DISTRO/setup.bash" >&2; return 1; }
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH GZ_SIM_RESOURCE_PATH RGL_PATTERNS_DIR
source "/opt/ros/$SCUT_ROS_DISTRO/setup.bash"
export SCUT_ROOT ROS_DOMAIN_ID="${SCUT_ROS_DOMAIN_ID:-$scut_default_domain}"
export GZ_PARTITION="${SCUT_GZ_PARTITION:-odin_navigation}"
export SCUT_INSTALL="$SCUT_ROOT/install/$SCUT_PROFILE"
unset scut_default_ros scut_default_domain
