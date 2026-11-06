#!/usr/bin/env bash
set -eo pipefail
export SCUT_PROFILE=sim
source "$(dirname "$0")/env.sh"
source "$SCUT_INSTALL/local_setup.bash"
exec ros2 launch uav_bringup simulation.launch.py root:="$SCUT_ROOT" "$@"
