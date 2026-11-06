#!/usr/bin/env bash
set -eo pipefail
replay_uri=""
replay_rate=0.5
replay_offset=0
replay_loop=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate|--start-offset)
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      if [[ "$1" == --rate ]]; then replay_rate="$2"; else replay_offset="$2"; fi
      shift 2 ;;
    --once) replay_loop=false; shift ;;
    --help|-h)
      echo 'Usage: bash scripts/play_bag.sh BAG_DIRECTORY_OR_DB3 [--rate 0.5] [--start-offset SECONDS] [--once]'
      echo 'Opens RViz2 and replays visualization topics in isolated domain 193 by default.'
      echo 'Default loops; --once plays once and leaves RViz open. Close RViz or Ctrl+C to stop.'
      echo 'An unfinished SQLite bag can be viewed by specifying its nonempty .db3 file directly.'
      exit 0 ;;
    -*) echo "Unknown option: $1" >&2; exit 2 ;;
    *) [[ -z "$replay_uri" ]] || { echo 'Only one bag path is supported.' >&2; exit 2; }
       replay_uri="$1"; shift ;;
  esac
done
[[ -n "$replay_uri" ]] || { echo 'Specify a bag directory or a nonempty .db3 file (--help).' >&2; exit 2; }
export SCUT_ROS_DOMAIN_ID="${SCUT_REPLAY_DOMAIN_ID:-193}"
python3 - "$SCUT_ROS_DOMAIN_ID" "$replay_rate" "$replay_offset" <<'PY'
import math,sys
try:
    domain=int(sys.argv[1]);rate=float(sys.argv[2]);offset=float(sys.argv[3])
    assert 1 <= domain <= 232 and math.isfinite(rate) and rate>0 and math.isfinite(offset) and offset>=0
except (ValueError,AssertionError):
    raise SystemExit('Use replay domain 1..232 (never hardware domain 0), positive rate and nonnegative offset.')
PY
source "$(dirname "$0")/env.sh"
source "$SCUT_INSTALL/local_setup.bash"
replay_uri="$(realpath "$replay_uri")"
storage_options=()
if [[ -f "$replay_uri" && "$replay_uri" == *.db3 && -s "$replay_uri" ]]; then
  storage_options=(-s sqlite3)
elif [[ ! -f "$replay_uri/metadata.yaml" ]]; then
  echo 'No bag metadata found. For an unfinished bag, specify a nonempty .db3 file.' >&2
  exit 1
fi
mkdir -p "$SCUT_ROOT/run/replay"
exec 9>"$SCUT_ROOT/run/replay/domain-$ROS_DOMAIN_ID.lock"
flock -n 9 || { echo "A replay in domain $ROS_DOMAIN_ID is already running." >&2; exit 1; }
replay_log="$SCUT_ROOT/run/replay/$(date +%Y%m%d-%H%M%S-%N)"
mkdir -p "$replay_log"
rviz_pid="";player_pid=""
cleanup() {
  [[ -z "$player_pid" ]] || kill -INT "$player_pid" 2>/dev/null || true
  [[ -z "$rviz_pid" ]] || kill -TERM "$rviz_pid" 2>/dev/null || true
  [[ -z "$player_pid" ]] || wait "$player_pid" 2>/dev/null || true
  [[ -z "$rviz_pid" ]] || wait "$rviz_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
topics=(/tf /tf_static /navigation/odometry /navigation/cloud /navigation/flight_path
 /navigation/state_markers /navigation/status /navigation/health /navigation/sensor_pose
 /navigation/staged_goals /planning/a_star_list /planning/init_list /planning/optimal_list
 /planning/accepted /planning/global_list /planning/failed_list /planning/goal_point
 /planning/diagnostics /planning/runtime_diagnostics /planning/position_cmd
 /planning/trajectory /planning/execution_status /grid_map/bounds
 /grid_map/occupancy /grid_map/occupancy_inflate /odin1/cloud_slam)
loop_options=()
if $replay_loop; then loop_options=(--loop); fi
rviz2 -d "$SCUT_ROOT/config/real/replay.rviz" --ros-args -p use_sim_time:=true > "$replay_log/rviz.log" 2>&1 &
rviz_pid=$!
ros2 bag play "$replay_uri" "${storage_options[@]}" --clock 50 --rate "$replay_rate" \
 --start-offset "$replay_offset" --delay 2 --disable-keyboard-controls "${loop_options[@]}" \
 --qos-profile-overrides-path "$SCUT_ROOT/config/record_qos.yaml" \
 --topics "${topics[@]}" > "$replay_log/player.log" 2>&1 &
player_pid=$!
printf 'RViz replay: domain=%s rate=%s offset=%ss\nBag: %s\nLogs: %s\nClose RViz or Ctrl+C to stop.\n' "$ROS_DOMAIN_ID" "$replay_rate" "$replay_offset" "$replay_uri" "$replay_log"
printf 'rviz_pid=%s\nplayer_pid=%s\n' "$rviz_pid" "$player_pid" > "$replay_log/pids.txt"
rc=0
wait -n "$rviz_pid" "$player_pid" || rc=$?
if kill -0 "$rviz_pid" 2>/dev/null && ! $replay_loop && [[ "$rc" == 0 ]]; then
  echo 'Playback finished. RViz remains open on the last frame.'
  wait "$rviz_pid" || true
elif [[ "$rc" != 0 ]]; then
  echo "Replay exited with code $rc; see $replay_log" >&2
fi
exit "$rc"
