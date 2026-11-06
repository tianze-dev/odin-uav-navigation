#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/record.sh [--real] [extra topics / ros2 bag record options]'
  echo 'Start before flight; Ctrl+C finalizes the bag and a review summary.'
  echo 'Bags: run/bags/<id>; config, live parameters, events and summary: run/metadata/<id>.'
  echo 'Defaults: split every 120 s or 1 GiB; MCAP with sqlite3 fallback for --real.'
  echo 'Cache: 16 MiB; sqlite3 uses resilient WAL/FULL storage settings.'
  exit 0
fi
real_record=false
if [[ "${1:-}" == --real ]]; then
  real_record=true
  export SCUT_PROFILE=real
  export SCUT_ROS_DOMAIN_ID="${SCUT_ROS_DOMAIN_ID:-0}"
  shift
fi
source "$(dirname "$0")/env.sh"
source "$SCUT_INSTALL/local_setup.bash"
record_id="$(date +%Y%m%d-%H%M%S-%N)"
record_meta="$SCUT_ROOT/run/metadata/$record_id"
mkdir -p "$SCUT_ROOT/run/bags" "$record_meta"
cp "$SCUT_ROOT/scripts/record.sh" "$SCUT_ROOT/scripts/lib/record_session.py" "$record_meta/"
cp -a "$SCUT_ROOT/config" "$record_meta/"
cp "$SCUT_ROOT/dependencies/lock.json" "$record_meta/"
record_px4_prefix="$(ros2 pkg prefix px4_msgs)"
if [[ -f "$record_px4_prefix/share/px4_msgs/source_manifest.json" ]]; then
  cp "$record_px4_prefix/share/px4_msgs/source_manifest.json" "$record_meta/px4_messages.json"
fi
if [[ -n "${SCUT_REAL_SOURCE_CONFIG:-}" ]]; then
  cp "$SCUT_REAL_SOURCE_CONFIG" "$record_meta/source_config.yaml"
fi
if [[ -n "${SCUT_REAL_NAVIGATION_CONFIG:-}" ]]; then
  cp "$SCUT_REAL_NAVIGATION_CONFIG" "$record_meta/navigation_config.yaml"
fi
git -C "$SCUT_ROOT" rev-parse --verify HEAD > "$record_meta/commit.txt" 2>/dev/null || printf 'uncommitted\n' > "$record_meta/commit.txt"
git -C "$SCUT_ROOT" diff HEAD -- > "$record_meta/worktree.diff"
git -C "$SCUT_ROOT" status --short > "$record_meta/worktree-status.txt"
printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\n' "$ROS_DOMAIN_ID" "$GZ_PARTITION" > "$record_meta/environment.txt"
storage="${SCUT_BAG_STORAGE:-mcap}"
if $real_record && [[ -z "${SCUT_BAG_STORAGE:-}" ]] && ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then storage=sqlite3; fi
storage_options=()
if [[ "$storage" == sqlite3 ]]; then
  storage_options=(--storage-preset-profile resilient
    --storage-config-file "$SCUT_ROOT/config/record_sqlite.yaml")
fi
topics=(/clock /navigation/odometry /navigation/cloud /navigation/sensor_pose /navigation/health
  /navigation/goal /navigation/rviz_goal /navigation/waypoints /navigation/staged_goals
  /navigation/flight_path /navigation/state_markers
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/bounds
  /planning/goal_point /planning/global_list /planning/init_list /planning/optimal_list
  /planning/a_star_list /planning/failed_list /planning/accepted
  /navigation/status /navigation/control_status /navigation/control_diagnostics /navigation/localization_fault
  /planning/runtime_diagnostics /planning/diagnostics /planning/duration_ms /planning/trajectory /planning/safety /planning/execution_status
  /planning/position_cmd /planning/executor_fault /tf /tf_static /parameter_events /rosout
  /odin1/cloud_raw /odin1/cloud_slam /odin1/imu
  /odin1/odometry_highfreq /odin1/odometry /odin1/wiwc)
if $real_record; then
  topics+=(/fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint
    /fmu/in/offboard_control_mode /fmu/in/vehicle_command /fmu/out/vehicle_command_ack
    /fmu/out/vehicle_status /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry
    /fmu/out/vehicle_land_detected /fmu/out/estimator_status_flags /fmu/out/failsafe_flags
    /fmu/out/timesync_status)
fi
exec python3 "$SCUT_ROOT/scripts/lib/record_session.py" \
  --metadata "$record_meta" --bag "$SCUT_ROOT/run/bags/$record_id" -- \
  ros2 bag record -s "$storage" -o "$SCUT_ROOT/run/bags/$record_id" \
  -b "${SCUT_RECORD_SPLIT_BYTES:-1073741824}" -d "${SCUT_RECORD_SPLIT_SECONDS:-120}" \
  --max-cache-size "${SCUT_RECORD_CACHE_BYTES:-16777216}" "${storage_options[@]}" \
  --qos-profile-overrides-path "$SCUT_ROOT/config/record_qos.yaml" "$@" "${topics[@]}"
