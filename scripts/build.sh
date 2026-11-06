#!/usr/bin/env bash
set -eo pipefail
export SCUT_PROFILE=real
if [[ "${1:-}" == --sim ]]; then export SCUT_PROFILE=sim; shift
elif [[ "${1:-}" == --real ]]; then shift
elif [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/build.sh [--real|--sim] [-- colcon options]'; exit 0
fi
if [[ "${1:-}" == -- ]]; then shift
elif [[ $# -gt 0 ]]; then echo 'Use -- before colcon options' >&2; exit 2; fi
source "$(dirname "$0")/env.sh"
cd "$SCUT_ROOT"
bases=()
for directory in src/*; do
  [[ -d "$directory" ]] || continue
  [[ "$directory" == src/px4_msgs ]] && continue
  [[ "$SCUT_PROFILE" == real && "$directory" == src/uav_description ]] && continue
  bases+=("$directory")
done
if [[ "$SCUT_PROFILE" == real ]]; then
  python3 scripts/verify_sources.py
  bases+=(src/px4_msgs)
else
  [[ "$SCUT_ROS_DISTRO" == jazzy ]] || { echo 'Simulation requires Jazzy / Gazebo Harmonic' >&2; exit 1; }
  python3 scripts/generate/prepare_messages.py
  python3 simulation/odin_simulation_gz/scripts/fetch_dependencies.py
  bases+=(generated/simulation/px4_msgs simulation/odin_simulation_gz)
fi
colcon --log-base "log/$SCUT_PROFILE" build --base-paths "${bases[@]}" \
  --build-base "build/$SCUT_PROFILE" --install-base "$SCUT_INSTALL" --symlink-install \
  --parallel-workers "${SCUT_BUILD_JOBS:-2}" "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
if [[ "$SCUT_PROFILE" == sim ]]; then
  source "$SCUT_INSTALL/local_setup.bash"
  python3 scripts/generate/prepare_sim_model.py
  python3 scripts/generate/prepare_rviz_model.py
fi
