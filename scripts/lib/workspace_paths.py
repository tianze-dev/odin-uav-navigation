"""读取本机仿真依赖路径；真实驱动不由本工程管理。"""
import json, os
from pathlib import Path
ROOT = Path(os.environ.get("SCUT_ROOT", Path(__file__).resolve().parents[2])).resolve()
def px4_root():
    local_file = ROOT / "dependencies/local.json"
    local = json.loads(local_file.read_text()) if local_file.exists() else {}
    p = Path(os.environ.get("PX4_ROOT") or local.get("PX4_ROOT") or ROOT / ".deps/PX4-Autopilot").expanduser().resolve()
    if not (p / "msg").is_dir():
        raise RuntimeError("Set PX4_ROOT or dependencies/local.json to the PX4 simulation checkout")
    return p
