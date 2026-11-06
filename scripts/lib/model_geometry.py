"""Derive simulation adapter geometry from the actual composed SDF model."""

import math, xml.etree.ElementTree as ET
import numpy as np
from uav_state_adapter.core import quaternion
from sensor_profiles import sensor_profile


def pose(text):
    x, y, z, r, p, a = map(float, (text or "0 0 0 0 0 0").split())
    cr, sr, cp, sp, ca, sa = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(a),
        math.sin(a),
    )
    R = np.array(
        [
            [ca * cp, ca * sp * sr - sa * cr, ca * sp * cr + sa * sr],
            [sa * cp, sa * sp * sr + ca * cr, sa * sp * cr - ca * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    return np.array([x, y, z]), R


def frame_pose(model, name, reference="base_link"):
    """解析生成模型中的 link/joint/frame 位姿链，返回 reference 下的位姿。"""
    elements = {e.get("name"): e for e in model if e.tag in ("link", "joint", "frame")}

    def resolve(current, visiting):
        if current == reference:
            return np.zeros(3), np.eye(3)
        if current in visiting or current not in elements:
            raise ValueError(f"Unresolved or cyclic SDF frame: {current}")
        element = elements[current]
        node = element.find("pose")
        default = (element.findtext("child") if element.tag == "joint" else
                   element.get("attached_to", "__model__"))
        parent = node.get("relative_to", default) if node is not None else default
        t, r = pose(node.text if node is not None else None)
        pt, pr = resolve(parent, visiting | {current})
        return pt + pr @ t, pr @ r

    return resolve(name, set())


def simulation_geometry(px4, model, sensor_name="odin1"):
    profile = sensor_profile(sensor_name)
    base = (
        ET.parse(px4 / "Tools/simulation/gz/models/x500_base/model.sdf")
        .getroot()
        .find("model")
    )
    tb, rb = pose(base.findtext("pose"))
    composed = ET.parse(model).getroot().find("model")
    link = composed.find(f"link[@name='{profile['lidar_link']}']")
    if link is None:
        raise ValueError("Generated X500 is missing the selected lidar link; rebuild")
    sensor = link.find(f"sensor[@name='{profile['depth_sensor']}']")
    if sensor is None:
        raise ValueError("Generated X500 is missing the RGL sensor")
    sensor_pose = sensor.find("pose")
    relative = sensor_pose.get("relative_to", profile["lidar_link"]) if sensor_pose is not None else profile["lidar_link"]
    tl, rl = frame_pose(composed, relative)
    ts, rs = pose(sensor.findtext("pose"))
    radius = 0.0
    for rotor in base.findall("link"):
        if not rotor.get("name", "").startswith("rotor_"):
            continue
        t, _ = pose(rotor.findtext("pose"))
        size = rotor.findtext("collision/geometry/box/size")
        if size is None:
            raise ValueError("Unknown propeller collision geometry")
        dims = np.array(list(map(float, size.split())))
        radius = max(radius, float(np.linalg.norm(t) + 0.5 * np.linalg.norm(dims[:2])))
    if radius <= 0:
        raise ValueError("No rotor geometry found")
    return {
        "odom_sensor_translation": list(map(float, -rb.T @ tb)),
        "odom_sensor_quaternion": list(map(float, quaternion(rb.T))),
        "sensor_translation": list(map(float, tl + rl @ ts)),
        "sensor_quaternion": list(map(float, quaternion(rl @ rs))),
        "cloud_frame": sensor.findtext("plugin/frame"),
    }, radius + 0.1


def simulation_envelope(px4, model, margin=0.05, tilt_degrees=15.0):
    """Yaw-independent collision envelope about base_link, including spinning props.

    Bound every collision primitive over roll/pitch tilt up to tilt_degrees.
    Margin is additional to the voxel-box quantization handled by GridMap.
    """
    import itertools
    if not math.isfinite(margin) or margin < 0 or not 0 <= tilt_degrees <= 45:
        raise ValueError('Invalid simulation envelope margin or tilt')
    base = ET.parse(px4 / 'Tools/simulation/gz/models/x500_base/model.sdf').find('model')
    composed = ET.parse(model).find('model')
    supports = []
    for tree in (base, composed):
        for link in tree.findall('link'):
            if not link.findall('collision'):
                continue
            t, r = (pose(link.findtext('pose')) if tree is base else
                    frame_pose(composed, link.get('name')))
            for collision in link.findall('collision'):
                ct, cr = pose(collision.findtext('pose'))
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    half = np.array(list(map(float, geometry.findtext('box/size').split()))) / 2
                elif geometry.find('cylinder') is not None:
                    half = np.array([float(geometry.findtext('cylinder/radius'))] * 2 +
                                    [float(geometry.findtext('cylinder/length')) / 2])
                else:
                    raise ValueError('Unsupported collision shape in simulation envelope')
                if link.get('name').startswith('rotor_'):
                    # All rotor phases, not just the slender blade at zero angle.
                    centre = t + r @ ct
                    radial = np.linalg.norm(centre[:2]) + np.linalg.norm(half[:2])
                    supports.extend((radial, centre[2] + z) for z in (-half[2], half[2]))
                else:
                    for signs in itertools.product((-1, 1), repeat=3):
                        point = t + r @ (ct + cr @ (half * signs))
                        supports.append((np.linalg.norm(point[:2]), point[2]))
    tilt = math.radians(tilt_degrees)
    def vertical(radial, z):
        angle = min(tilt, max(0., math.atan2(radial, z)))
        return z * math.cos(angle) + radial * math.sin(angle)
    def horizontal(radial, z):
        angle = min(tilt, math.atan2(abs(z), radial))
        return radial * math.cos(angle) + abs(z) * math.sin(angle)
    if not supports:
        raise ValueError('Missing collision geometry')
    return {
        'radius': float(max(horizontal(r, z) for r, z in supports) + margin),
        'above': float(max(vertical(r, z) for r, z in supports) + margin),
        'below': float(max(vertical(r, -z) for r, z in supports) + margin),
        'margin': margin, 'tilt_degrees': tilt_degrees,
    }
