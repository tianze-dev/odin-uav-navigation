"""仿真传感器的模型与接口约定；每次启动只选择一个传感器。"""

PROFILES = {
    "odin1": {
        "package": "odin_simulation_gz",
        "model": "x500_odin1",
        "wrapper": "odin1_mount.urdf.xacro",
        "mount_joint": "odin1_odin_mount",
        "links": ("odin1_odin_link", "odin1_lidar", "odin1_imu", "odin1_camera_link"),
        "lidar_link": "odin1_lidar",
        "depth_sensor": "odin1_depth",
        "cloud_topic": "/odin1/cloud_raw",
        "imu_topic": "/odin1/imu",
        "default_yaw": "follow",
    },
}


def sensor_profile(name):
    if name not in PROFILES:
        raise ValueError("sensor must be odin1")
    return PROFILES[name].copy()
