#!/usr/bin/env bash
set -eo pipefail
export SCUT_ROS_DOMAIN_ID="${SCUT_ROS_DOMAIN_ID:-0}"
export SCUT_PROFILE=real
source "$(dirname "$0")/env.sh"
source "$SCUT_INSTALL/local_setup.bash"
exec python3 "$SCUT_ROOT/scripts/lib/odin_preflight.py" "$@"
