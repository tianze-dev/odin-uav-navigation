"""获取指定 PX4 仿真源码；已有检出不重置。"""
from pathlib import Path
import json, subprocess
root = Path(__file__).resolve().parents[2]
spec = json.loads((root / "dependencies/lock.json").read_text())["px4_simulation"]
target = root / ".deps/PX4-Autopilot"
if target.exists():
    raise SystemExit("PX4 directory exists; inspect it or select PX4_ROOT explicitly")
subprocess.run(["git", "clone", "--filter=blob:none", spec["url"], str(target)], check=True)
subprocess.run(["git", "-C", str(target), "checkout", "--detach", spec["commit"]], check=True)
subprocess.run(["git", "-C", str(target), "submodule", "update", "--init", "--recursive"], check=True)
