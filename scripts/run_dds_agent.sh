#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: PX4_SERIAL_DEV=/dev/px4_fcu PX4_SERIAL_BAUD=921600 bash scripts/run_dds_agent.sh'
  echo 'Use an existing DDS service OR this script, not two agents on the same serial port.'
  exit 0
fi
source "$(dirname "$0")/env.sh"
agent="${XRCE_AGENT_BIN:-$SCUT_ROOT/.deps/agent/bin/MicroXRCEAgent}"
if [[ ! -x "$agent" ]]; then agent="$(command -v MicroXRCEAgent || true)"; fi
[[ -n "$agent" && -x "$agent" ]] || { echo 'Build Agent v2.4.2 with scripts/setup/install_agent.sh first.' >&2; exit 1; }
python3 - <<'PY'
from pathlib import Path
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:cmd=(p/'cmdline').read_bytes().split(b'\0')
 except OSError:continue
 if cmd and Path(cmd[0].decode(errors='replace')).name=='MicroXRCEAgent':
  raise SystemExit(f'An Agent is already running (PID {p.name}); preserve the unique existing instance.')
PY
device="${PX4_SERIAL_DEV:-/dev/px4_fcu}"
[[ -e "$device" ]] || { echo "PX4 serial device is absent: $device" >&2; exit 1; }
export LD_LIBRARY_PATH="$SCUT_ROOT/.deps/agent/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$agent" serial --dev "$device" -b "${PX4_SERIAL_BAUD:-921600}"
