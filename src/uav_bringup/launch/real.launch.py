"""One real localization source. Driver startup remains explicit and external."""

import os
import json
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    config = Path(get_package_share_directory("uav_bringup")) / "config"
    sensor = get("sensor")
    mode = get("mode")
    if sensor != "odin1" or mode not in ["real", "observe", "replay"]:
        raise ValueError("Invalid real/replay profile")
    enabled = get("enable_control")
    if enabled not in ("true", "false"):
        raise ValueError("enable_control must be true or false")
    enable_control = enabled == "true"
    if enable_control and mode != "real":
        raise ValueError("Control may only be explicitly enabled in mode:=real")
    external_enabled = get("enable_external_odometry")
    if external_enabled not in ("true", "false"):
        raise ValueError("enable_external_odometry must be true or false")
    publish_external = external_enabled == "true"
    if publish_external and (mode != "real" or sensor != "odin1"):
        raise ValueError("Native external odometry requires sensor:=odin1 mode:=real")
    if enable_control and sensor == "odin1" and not publish_external:
        raise ValueError("Odin control requires enable_external_odometry:=true")
    use_sim_time = mode == "replay"
    common = {"use_sim_time": use_sim_time}
    source = Path(get("source_config") or config / "real" / f"{sensor}.yaml").expanduser()
    profile = Path(get("navigation_config") or config / "real" / f"{sensor}_navigation.yaml").expanduser()
    speed_profile = get("speed_profile")
    if speed_profile not in ("conservative", "baseline", "aggressive"):
        raise ValueError("speed_profile must be conservative, baseline or aggressive")
    # 显式速度档覆盖导航配置；aggressive 直接复用仿真的规划/执行段。
    planner_speed_params = []
    executor_speed_params = []
    if speed_profile == "baseline":
        planner_speed_params = [str(config / "real" / f"{sensor}_speed_1m.yaml")]
        executor_speed_params = list(planner_speed_params)
    elif speed_profile == "aggressive":
        aggressive = yaml.safe_load((config / "profiles/aggressive.yaml").read_text())
        planner_speed_params = [aggressive["planner"], aggressive["execution"]]
        executor_speed_params = [aggressive["execution"]]
    source_params = yaml.safe_load(source.read_text())["uav_state_adapter"]["ros__parameters"]
    if publish_external and source_params.get("calibrated") is not True:
        raise ValueError("Odin external odometry requires measured extrinsics and calibrated: true in source_config")
    if mode != "replay":
        # 防止误用 SITL 消息覆盖已验证的 PX4 1.15 实机接口。
        manifest_path = Path(get_package_share_directory("px4_msgs")) / "source_manifest.json"
        if not manifest_path.is_file():
            raise RuntimeError("Use bash scripts/build.sh --real; installed px4_msgs has no provenance")
        manifest = json.loads(manifest_path.read_text())
        root = Path(get("root"))
        expected = json.loads((root / "dependencies/lock.json").read_text())["px4_real_msgs"]["commit"]
        if manifest.get("profile") != "real" or manifest.get("commit") != expected:
            raise RuntimeError("Installed px4_msgs does not match the pinned real flight interface")
    params = [
        str(source),
        {"mode": "replay" if use_sim_time else "real", "sensor": sensor, **common},
    ]
    nodes = [
        Node(
            package="uav_state_adapter",
            executable="state_adapter",
            parameters=params,
            output="screen",
        ),
        Node(
            package="ego_planner",
            executable="ego_planner_node",
            parameters=[
                str(config / "planner.yaml"),
                str(profile), *planner_speed_params, common,
            ],
            remappings=[
                ("grid_map/cloud", "/navigation/cloud"),
                ("grid_map/odom", "/navigation/odometry"),
                ("grid_map/sensor_pose", "/navigation/sensor_pose"),
            ],
            output="screen",
        ),
    ]
    if enable_control:
        nodes.append(Node(package="uav_px4_bridge", executable="traj_server",
                          parameters=[str(profile), *executor_speed_params, common], output="screen"))
    if mode != "replay":
        nodes.append(Node(
            package="uav_px4_bridge", executable="px4_bridge",
            parameters=[str(profile), common, {
                "mode": "real", "px4_namespace": get("px4_namespace"),
                "publish_control": enable_control,
                "publish_external_odometry": publish_external,
                "offboard_entry": "rc",
            }], output="screen",
        ))
    if get("rviz") == "true":
        # 显示与实际地图相同的包络，兼容本地 navigation_config 覆盖。
        planner = yaml.safe_load((config / "planner.yaml").read_text())["ego_planner_node"]["ros__parameters"]
        planner.update(yaml.safe_load(profile.read_text()).get("ego_planner_node", {}).get("ros__parameters", {}))
        radius = planner["grid_map/obstacles_inflation"]
        nodes.append(Node(package="uav_rviz_plugins", executable="navigation_visuals",
                          parameters=[common, {
                              "vehicle_label": "UAV",
                              "body_shape": planner.get("grid_map/inflation_shape", "box"),
                              "body_radius": radius,
                              "body_above": planner.get("grid_map/body_above", radius),
                              "body_below": planner.get("grid_map/body_below", radius),
                          }], output="screen"))
        nodes.append(Node(package="rviz2", executable="rviz2", parameters=[common],
                          arguments=["-d", str(config / "real/odin1.rviz")], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "root", default_value=os.environ.get("SCUT_ROOT", "")
            ),
            DeclareLaunchArgument("sensor", default_value="odin1"),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("enable_control", default_value="false"),
            DeclareLaunchArgument("enable_external_odometry", default_value="false",
                                  description="Publish Odin body odometry to PX4; requires calibrated real configuration"),
            DeclareLaunchArgument("speed_profile", default_value="conservative",
                                  description="conservative (0.5 m/s), baseline (1 m/s), aggressive (2.2 m/s)"),
            DeclareLaunchArgument("source_config", default_value=os.environ.get("SCUT_REAL_SOURCE_CONFIG", "")),
            DeclareLaunchArgument("navigation_config", default_value=os.environ.get("SCUT_REAL_NAVIGATION_CONFIG", "")),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("px4_namespace", default_value=""),
            OpaqueFunction(function=setup),
        ]
    )
