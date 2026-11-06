#!/usr/bin/env python3
"""将所选传感器的完整子树安装到 PX4 X500，不修改外部 PX4。"""

import argparse
import copy
from pathlib import Path
import sys

import subprocess
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from workspace_paths import ROOT, px4_root
from sensor_profiles import sensor_profile

PX4 = px4_root()

def prepare(sensor_name="odin1", destination=None):
    profile = sensor_profile(sensor_name)
    model_name = profile["model"]
    wrapper = ROOT / "src/uav_description/urdf" / profile["wrapper"]
    with tempfile.TemporaryDirectory(prefix=f"{model_name}_mount_") as tmp:
        urdf = Path(tmp) / "mount.urdf"
        subprocess.run(["xacro", str(wrapper), "-o", str(urdf)], check=True)
        result = subprocess.run(
            ["gz", "sdf", "-p", str(urdf)], check=True, capture_output=True, text=True
        )
    mount = ET.fromstring(result.stdout).find("model")
    required = set(profile["links"])
    if not required <= {link.get("name") for link in mount.findall("link")}:
        raise ValueError("Conversion lost preserved sensor links")
    if mount.find(f"joint[@name='{profile['mount_joint']}']") is None:
        raise ValueError("Conversion lost preserved mounting joint")
    # 保留转换器生成的 relative_to 位姿链；不对新版模型再次进行 CAD 修正。
    base = ET.parse(PX4 / "Tools/simulation/gz/models/x500/model.sdf")
    root = base.getroot()
    root.set("version", "1.9")
    model = root.find("model")
    model.set("name", model_name)
    odometry = ET.Element(
        "plugin",
        {
            "filename": "gz-sim-odometry-publisher-system",
            "name": "gz::sim::systems::OdometryPublisher",
        },
    )
    for tag, value in (
        ("dimensions", "3"),
        ("odom_topic", f"/{model_name}/odometry"),
        ("odom_covariance_topic", "/simulation/scut_odometry_covariance"),
        ("odom_frame", "map"),
        ("robot_base_frame", "model_origin"),
        ("odom_publish_frequency", "100"),
    ):
        ET.SubElement(odometry, tag).text = value
    model.append(odometry)
    for child in mount:
        if child.tag == "link" and child.get("name") == "base_link":
            continue  # x500 owns the actual body inertia
        model.append(copy.deepcopy(child))
    root.insert(
        0,
        ET.Comment(f"Generated from {profile['package']} using {profile['wrapper']}."),
    )
    ET.indent(base)
    output = Path(destination or ROOT / "generated/models" / model_name / "model.sdf")
    output.parent.mkdir(parents=True, exist_ok=True)
    base.write(output, encoding="utf-8", xml_declaration=True)
    (output.parent / "model.config").write_text(
        f'<?xml version="1.0"?><model><name>{model_name}</name><version>1.0</version>'
        f'<sdf version="1.9">model.sdf</sdf><description>X500 with {sensor_name}</description></model>'
    )
    print(f"Prepared {output}")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor", choices=["odin1"], default="odin1")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    prepare(args.sensor, args.output)
