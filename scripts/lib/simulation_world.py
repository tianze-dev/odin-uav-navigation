"""为所选传感器准备唯一的场景管理器，不修改场景源文件。"""

import atexit
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


MANAGERS = {"odin1": ("OdinRGLSceneManager", "odin_sim::RGLServerPluginManager")}

def prepare_sensor_world(source, sensor, plugin_path, patterns_dir=None):
    if sensor not in MANAGERS:
        raise ValueError("sensor must be odin1")
    libraries = ("libOdinRGLSceneManager.so", "libOdinRGLDepthSensor.so")
    required = [Path(plugin_path) / name for name in (*libraries, "libRobotecGPULidar.so")]
    if missing := [str(p) for p in required if not p.is_file()]:
        raise RuntimeError(f"Rebuild the selected sensor; missing: {missing}")
    tree = ET.parse(source)
    worlds = tree.getroot().findall("world")
    if len(worlds) != 1:
        raise ValueError("Expected exactly one simulation world")
    world = worlds[0]
    filename, name = MANAGERS[sensor]
    for plugin in list(world.findall("plugin")):
        if plugin.get("name") in {n for _, n in MANAGERS.values()} and plugin.get("name") != name:
            world.remove(plugin)
    systems = [(filename, name), ("gz-sim-imu-system", "gz::sim::systems::Imu")]
    if sensor == "odin1":
        systems.append(("gz-sim-sensors-system", "gz::sim::systems::Sensors"))
        physics = world.find("physics")
        if physics is None:
            physics = ET.SubElement(world, "physics", name="physics", type="ignored")
        step = physics.find("max_step_size")
        if step is None:
            step = ET.SubElement(physics, "max_step_size")
        # 400 Hz public IMU = five genuine 0.5 ms physics samples.
        step.text = "0.0005"
    for filename, name in systems:
        existing = world.findall(f"plugin[@name='{name}']")
        if len(existing) > 1:
            raise ValueError(f"Duplicate world system: {name}")
        plugin = existing[0] if existing else ET.SubElement(world, "plugin", filename=filename, name=name)
        if name == MANAGERS[sensor][1]:
            flag = plugin.find("do_ignore_entities_in_lidar_link")
            if flag is None:
                flag = ET.SubElement(plugin, "do_ignore_entities_in_lidar_link")
            flag.text = "true"
        if name == "gz::sim::systems::Sensors":
            engine = plugin.find("render_engine")
            if engine is None:
                engine = ET.SubElement(plugin, "render_engine")
            engine.text = "ogre2"
    with tempfile.NamedTemporaryFile(prefix=f"scut_{sensor}_", suffix=".sdf", delete=False) as output:
        tree.write(output, encoding="utf-8", xml_declaration=True)
        path = Path(output.name)
    atexit.register(path.unlink, missing_ok=True)
    return path
