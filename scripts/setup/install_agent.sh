#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/../env.sh"
root="$SCUT_ROOT/.deps/Micro-XRCE-DDS-Agent"
if [[ ! -d "$root/.git" ]]; then
  git clone --branch v2.4.2 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$root"
fi
[[ "$(git -C "$root" describe --tags --exact-match)" == v2.4.2 ]] || { echo 'Unexpected Agent source version' >&2; exit 1; }
cmake -S "$root" -B "$SCUT_ROOT/.deps/agent-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SCUT_ROOT/.deps/agent"
cmake --build "$SCUT_ROOT/.deps/agent-build" -j "${SCUT_BUILD_JOBS:-2}"
cmake --install "$SCUT_ROOT/.deps/agent-build"
