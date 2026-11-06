"""SCUT simulation only: new EGO control stack, extracted environment assets."""

import os, sys, socket
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    root = Path(arg("root")).resolve()
    import json
    manifest = json.loads((Path(get_package_share_directory("px4_msgs")) / "source_manifest.json").read_text())
    expected = json.loads((root / "dependencies/lock.json").read_text())["px4_simulation"]["commit"]
    if manifest.get("profile") != "simulation" or manifest.get("commit") != expected:
        raise RuntimeError("Build and source the simulation profile with scripts/build.sh --sim")

    share = Path(get_package_share_directory("uav_bringup"))
    sys.path.insert(0, str(share / "runtime"))
    os.environ["SCUT_ROOT"] = str(root)
    from workspace_paths import px4_root
    from flight_profile import load_flight_profile
    profile = load_flight_profile(root, arg("flight_profile"), config_root=share / "config")
    from sensor_profiles import sensor_profile
    sensor_name = arg("sensor")
    sensor_spec = sensor_profile(sensor_name)
    model_name = sensor_spec["model"]
    model_dir = root / "generated/models" / model_name
    yaw_mode = sensor_spec["default_yaw"] if arg("yaw_mode") == "auto" else arg("yaw_mode")
    if yaw_mode not in ("hold", "follow"):
        raise ValueError("yaw_mode must be auto, hold or follow")
    from simulation_options import world_source
    guidance_arg=arg("route_guidance")
    if guidance_arg not in ("auto","true","false"):
        raise ValueError("route_guidance must be auto, true or false")
    route_guidance=(profile["planner"].get("fsm/use_route_seed",False)
                    if guidance_arg=="auto" else guidance_arg=="true")

    px4 = px4_root()
    world_name = "x500_rmuc2026"
    instance = int(arg("instance"))
    port = int(arg("dds_port"))
    if not os.environ.get("GZ_PARTITION"):
        raise RuntimeError("Use scripts/run_sim.sh for isolated environment")
    # Refuse to share a DDS UDP port with another running project.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    finally:
        sock.close()
    try:
        sensor = Path(get_package_prefix(sensor_spec["package"]))
    except PackageNotFoundError as error:
        raise RuntimeError(f"Sensor backend {sensor_name} is not built. Run: "
                           "bash scripts/build.sh --sim") from error
    if not all((model_dir / name).is_file() for name in ("model.sdf", "robot.urdf")):
        raise RuntimeError(f"Missing generated model for {sensor_name}. Run: "
                           "bash scripts/build.sh --sim")
    plugin = sensor / "lib" / sensor_spec["package"] / "rgl"
    scene = arg("world")
    source = world_source(root, scene, arg("forest_density"), int(arg("forest_seed")))
    from simulation_world import prepare_sensor_world

    world = prepare_sensor_world(source, sensor_name, plugin)
    runtime = root / "run/simulation" / f"{scene}-{sensor_name}-{instance}"
    runtime.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(
        PX4_GZ_STANDALONE="1",
        PX4_GZ_WORLD=world_name,
        PX4_GZ_NO_FOLLOW="1",
        PX4_SIM_MODEL="gz_" + model_name,
        PX4_SYS_AUTOSTART="4001",
        PX4_GZ_MODELS=str(root / "generated/models"),
        PX4_UXRCE_DDS_NS="scut",
        PX4_UXRCE_DDS_PORT=str(port),
        PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0",
        GZ_SIM_RESOURCE_PATH=os.pathsep.join(
            [
                str(root / "generated/models"),
                str(px4 / "Tools/simulation/gz/models"),
                str(sensor / "share"),
                env.get("GZ_SIM_RESOURCE_PATH", ""),
            ]
        ),
        GZ_SIM_SYSTEM_PLUGIN_PATH=os.pathsep.join(
            [
                str(plugin),
                str(px4 / "build/px4_sitl_default/src/modules/simulation/gz_plugins"),
                env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ]
        ),
        PX4_PARAM_SIM_GZ_EN_ODOM="0",
        PX4_PARAM_SIM_GZ_EN_GPS="0",
        PX4_PARAM_EKF2_EV_CTRL="15",
        PX4_PARAM_EKF2_GPS_CTRL="0",
        PX4_PARAM_EKF2_HGT_REF="3",
        PX4_PARAM_COM_OBL_RC_ACT="4",
        PX4_PARAM_COM_OF_LOSS_T="0.5",
    )
    if arg("rviz_control") == "true":
        env.update(
            PX4_PARAM_COM_RC_IN_MODE="4",
            PX4_PARAM_NAV_DLL_ACT="0",
            PX4_PARAM_COM_ARM_WO_GPS="1",
        )
    env.update({"PX4_PARAM_" + key: str(value) for key, value in profile["px4"].items()})
    agent = root / ".deps/agent/bin/MicroXRCEAgent"
    agent_env = env.copy()
    agent_env["LD_LIBRARY_PATH"] = (
        str(root / ".deps/agent/lib") + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    )
    actions = [
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-r",
                *(["-s", "--headless-rendering"] if arg("headless") == "true" and sensor_name == "odin1"
                  else ["-s"] if arg("headless") == "true" else []),
                str(world),
            ],
            additional_env=env,
            output="log",
        ),
        ExecuteProcess(
            cmd=[str(agent), "udp4", "-p", str(port)],
            additional_env=agent_env,
            output="log",
        ),
        ExecuteProcess(
            cmd=[
                str(px4 / "build/px4_sitl_default/bin/px4"),
                "-i",
                str(instance),
                "-d",
                str(px4 / "build/px4_sitl_default/etc"),
            ],
            cwd=str(runtime),
            additional_env=env,
            output="log",
        ),
    ]
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"/{model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            ],
            remappings=[
                (f"/world/{world_name}/clock", "/clock"),
                (f"/{model_name}/odometry", "/simulation/model_odometry"),
            ],
            parameters=[{"use_sim_time": True}],
            output="log",
        )
    )
    from model_geometry import simulation_geometry, simulation_envelope

    geometry, _legacy_clearance = simulation_geometry(
        px4, model_dir / "model.sdf", sensor_name
    )
    envelope = simulation_envelope(px4, model_dir / "model.sdf", **profile["geometry"])
    description = (model_dir / "robot.urdf").read_text()
    common = {"use_sim_time": True}
    sensor_process = None
    if sensor_name == "odin1":
        from odin_simulation_gz.configuration import load_config
        odin_config = load_config(get_package_share_directory("odin_simulation_gz"))
        sensor_process = Node(
            package="odin_simulation_gz", executable="odin_ros_adapter", namespace="odin1",
            name="odin1_adapter", parameters=[common, {"gz_prefix": "/odin_sim/odin1",
                "frame_prefix": "odin1_", "rgb_hfov": float(odin_config["rgb_hfov"]),
                "rgb_vfov": float(odin_config["rgb_vfov"]), "imu_rate": float(odin_config["imu_rate"])}],
            additional_env=env, output="log",
        )
        actions.append(sensor_process)
    actions.extend(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[common, {"robot_description": description}],
                output="log",
            ),
            Node(
                package="uav_state_adapter",
                executable="state_adapter",
                parameters=[common, geometry, {"mode": "sim", "sensor": sensor_name, "cloud_topic": sensor_spec["cloud_topic"], "odom_topic": "/simulation/model_odometry", "source_frame": "map", "source_child": "model_origin", "cloud_in_world": False, "velocity_world": False, "calibrated": True}],
                output="log",
            ),
            Node(
                package="ego_planner",
                executable="ego_planner_node",
                parameters=[
                    str(share / "config/planner.yaml"),
                    profile["planner"], profile["execution"],
                    common,
                    {"fsm/allow_unknown": arg("allow_unknown") == "true",
                     "fsm/use_route_seed": route_guidance,
                     "grid_map/inflation_shape": "cylinder",
                     "grid_map/obstacles_inflation": envelope["radius"],
                     "grid_map/body_above": envelope["above"],
                     "grid_map/body_below": envelope["below"]},
                ],
                remappings=[
                    ("grid_map/cloud", "/navigation/cloud"),
                    ("grid_map/odom", "/navigation/odometry"),
                    ("grid_map/sensor_pose", "/navigation/sensor_pose"),
                ],
                output="log",
            ),
            Node(
                package="uav_px4_bridge",
                executable="traj_server",
                parameters=[common, profile["execution"],
                            {"execution/yaw_mode": yaw_mode}],
                output="log",
            ),
            Node(
                package="uav_px4_bridge",
                executable="px4_bridge",
                parameters=[common, {"mode": "sim", "px4_namespace": "/scut",
                    "manual_control_enabled": arg("rviz_control") == "true"}],
                output="log",
            ),
        ]
    )
    actions.append(Node(package="uav_rviz_plugins", executable="navigation_visuals",
                        parameters=[common, {"body_radius": envelope["radius"],
                                             "body_above": envelope["above"],
                                             "body_below": envelope["below"]}], output="log"))
    if arg("rviz") == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "config/navigation.rviz")],
                parameters=[common],
                output="log",
            )
        )
    # Only processes launched here are stopped. A planner exit is handled by bridge expiry;
    # sensor/physics exits end this simulation session.
    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=p,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="SCUT simulation process exited"))
                ],
            )
        )
        for p in [*actions[:4], *([sensor_process] if sensor_process is not None else [])]
    ]
    return handlers + actions


def generate_launch_description():
    defaults = {
        "root": os.environ.get("SCUT_ROOT", ""),
        "sensor": "odin1",
        "world": "test",
        "instance": "7",
        "dds_port": "8897",
        "headless": "false",
        "rviz": "true",
        "rviz_control": "true",
        "allow_unknown": "true",
        "flight_profile": "baseline",
        "forest_density": "medium",
        "forest_seed": "17",
        "route_guidance": "auto",
        "yaw_mode": "auto",
    }
    return LaunchDescription(
        [
            *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
            OpaqueFunction(function=setup),
        ]
    )
