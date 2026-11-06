#!/usr/bin/env python3
"""从同一 PX4 几何生成 RViz 整机 URDF；旋翼外观使用固定关节。"""
import argparse
from pathlib import Path
import sys

import xml.etree.ElementTree as ET
import xacro
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from workspace_paths import ROOT, px4_root
from sensor_profiles import sensor_profile


def prepare(sensor_name="odin1"):
    profile = sensor_profile(sensor_name)
    px4 = px4_root()
    model_dir = px4 / 'Tools/simulation/gz/models'
    base = ET.parse(model_dir / 'x500_base/model.sdf').find('model')
    robot = ET.fromstring(xacro.process_file(str(
        ROOT / 'src/uav_description/urdf' / profile['wrapper'])).toxml())
    robot.set('name', profile['model'])
    body = robot.find("link[@name='base_link']")
    for child in list(body):
        body.remove(child)
    for link in base.findall('link'):
        name = link.get('name')
        target = body if name == 'base_link' else ET.SubElement(robot, 'link', name=name)
        if name != 'base_link':
            joint = ET.SubElement(robot, 'joint', name='rviz_' + name + '_joint', type='fixed')
            ET.SubElement(joint, 'parent', link='base_link')
            ET.SubElement(joint, 'child', link=name)
            pose = (link.findtext('pose') or '0 0 0 0 0 0').split()
            ET.SubElement(joint, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
        for source in link.findall('visual'):
            # URDF 不支持 SDF 贴花平面；保留结构网格，略去三个板载标签。
            if source.find('geometry/plane') is not None:
                continue
            visual = ET.SubElement(target, 'visual', name=source.get('name'))
            pose = (source.findtext('pose') or '0 0 0 0 0 0').split()
            ET.SubElement(visual, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
            geometry = ET.SubElement(visual, 'geometry')
            mesh = source.find('geometry/mesh')
            if mesh is not None:
                uri = mesh.findtext('uri')
                path = model_dir / uri.removeprefix('model://')
                if not path.is_file():
                    raise FileNotFoundError(path)
                ET.SubElement(geometry, 'mesh', filename=path.as_uri(), scale=mesh.findtext('scale', '1 1 1'))
            else:
                shape = list(source.find('geometry'))[0]
                props = {part.tag: part.text for part in shape}
                if shape.tag not in ('box', 'cylinder', 'sphere'):
                    raise ValueError(f'Unexpected PX4 visual geometry: {shape.tag}')
                ET.SubElement(geometry, shape.tag, **props)
                color = source.findtext('material/diffuse', '0.35 0.35 0.35 1')
                if len(color.split()) == 3:
                    color += ' 1'
                material = ET.SubElement(visual, 'material', name=name + '_' + source.get('name'))
                ET.SubElement(material, 'color', rgba=color)
    output = ROOT / 'generated/models' / profile['model'] / 'robot.urdf'
    ET.indent(robot)
    output.write_text(ET.tostring(robot, encoding='unicode'))
    print(f'Prepared {output}')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor", choices=["odin1"], default="odin1")
    prepare(parser.parse_args().sensor)
