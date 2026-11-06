import hashlib, json, shutil, subprocess
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from workspace_paths import ROOT, px4_root

p = px4_root()
lock = json.loads((ROOT / "dependencies/lock.json").read_text())
head = subprocess.check_output(
    ["git", "-C", str(p), "rev-parse", "HEAD"], text=True
).strip()
if head != lock["px4_simulation"]["commit"]:
    raise RuntimeError("PX4 revision mismatch; update and validate lock explicitly")
if subprocess.check_output(["git", "-C", str(p), "status", "--porcelain", "--", "msg"], text=True).strip():
    raise RuntimeError("PX4 message source has local edits")
files = {f.stem: f for f in (p / "msg").rglob("*.msg")}
names = [
    "VehicleOdometry",
    "VehicleAttitude",
    "VehicleLocalPosition",
    "VehicleStatus",
    "VehicleLandDetected",
    "VehicleCommandAck",
    "VehicleCommand",
    "OffboardControlMode",
    "TrajectorySetpoint",
    "TimesyncStatus",
    "EstimatorStatusFlags",
    "FailsafeFlags",
]
out = ROOT / "generated/simulation/px4_msgs"
(out / "msg").mkdir(parents=True, exist_ok=True)
manifest = {}
while names:
    name = names.pop()
    if name in manifest:
        continue
    data = files[name].read_text()
    (out / "msg" / f"{name}.msg").write_text(data)
    manifest[name] = hashlib.sha256(data.encode()).hexdigest()
    for line in data.splitlines():
        words = line.split("#")[0].split()
        if words:
            dep = words[0].split("[")[0].split("/")[-1]
            if dep in files:
                names.append(dep)
(out / "source_manifest.json").write_text(
    json.dumps({"profile": "simulation", "commit": head, "messages": manifest}, indent=2)
)
(out / "package.xml").write_text(
    '<package format="3"><name>px4_msgs</name><version>0.1.0</version><description>Pinned PX4 wire definitions</description><maintainer email="uav@scut.example">SCUT UAV</maintainer><license>BSD-3-Clause</license><buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend><exec_depend>rosidl_default_runtime</exec_depend><member_of_group>rosidl_interface_packages</member_of_group><export><build_type>ament_cmake</build_type></export></package>'
)
(out / "CMakeLists.txt").write_text(
    'cmake_minimum_required(VERSION 3.16)\nproject(px4_msgs)\nfind_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\nfile(GLOB messages RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")\nrosidl_generate_interfaces(${PROJECT_NAME} ${messages})\ninstall(FILES source_manifest.json DESTINATION share/${PROJECT_NAME})\nament_export_dependencies(rosidl_default_runtime)\nament_package()\n'
)
shutil.copy2(p / "LICENSE", out / "LICENSE")
