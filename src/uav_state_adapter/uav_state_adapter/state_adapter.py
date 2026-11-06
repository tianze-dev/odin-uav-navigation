"""One selected source -> continuous body odometry and timestamped scan.
No localization algorithm is simulated: sim mode explicitly uses Gazebo truth.
"""

from collections import deque
import math
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster
from uav_interfaces.msg import NavigationHealth
from .core import body_state, body_covariances, rotation, quaternion, vector, fresh, ClockGuard
from .pointcloud import cloud_xyz
from .fault_recording import FaultRecorder


class StateAdapter(Node):
    def __init__(self, **node_options):
        super().__init__("uav_state_adapter", **node_options)
        defaults = {
            "mode": "real",
            "sensor": "odin1",
            "odom_topic": "/odin1/odometry_highfreq",
            "cloud_topic": "/odin1/cloud_slam",
            "source_frame": "odom",
            "source_child": "odin1_base_link",
            "sensor_translation": [0.0, 0.0, 0.0],
            "sensor_quaternion": [0.0, 0.0, 0.0, 1.0],
            "odom_sensor_translation": [0.0, 0.0, 0.0],
            "odom_sensor_quaternion": [0.0, 0.0, 0.0, 1.0],
            "velocity_world": True,
            "cloud_in_world": True,
            "calibrated": False,
            "odom_timeout": 0.25,
            "cloud_timeout": 0.5,
            "max_range": 30.0,
            "min_range": 0.5,
            "body_filter_radius": 0.0,
            "cloud_frame": "odom",
            "world_quaternion": [0.0, 0.0, 0.0, 1.0],
            "cloud_pose_tolerance": 0.015,
            "cloud_wait_timeout": 0.2,
            "require_cloud_for_odometry": False,
            "record_localization_faults": False,
            "localization_fault_directory": "",
        }
        self.p = {
            k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
            for k, v in defaults.items()
        }
        if self.p["mode"] not in ["sim", "real", "replay"] or self.p["sensor"] != "odin1":
            raise ValueError("invalid source selection")
        self.history = deque(maxlen=1200)
        if not math.isfinite(self.p["body_filter_radius"]) or self.p["body_filter_radius"] < 0:
            raise ValueError("body_filter_radius must be finite and nonnegative")
        self.world_rotation = rotation(self.p["world_quaternion"])
        self.pending_cloud = deque(maxlen=5)
        for name in ("cloud_pose_tolerance", "cloud_wait_timeout"):
            if not math.isfinite(self.p[name]) or not 0 < self.p[name] <= 0.5:
                raise ValueError("Invalid timestamp tolerance: " + name)
        self.epoch = time.time_ns() & ((1 << 63) - 1)
        self.last_odom = 0.0
        self.last_cloud = 0.0
        self.clock = ClockGuard()
        self.arrival = 0.0
        self.reason = "waiting for selected source"
        self.fault = False
        self.first_fault_reason = ""
        self.last_accepted_raw = None
        self.fault_pub = self.create_publisher(String, '/navigation/localization_fault',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.fault_recorder = None
        if self.p['record_localization_faults']:
            directory = (Path(self.p['localization_fault_directory'])
                         if self.p['localization_fault_directory'] else
                         Path(os.environ.get('SCUT_ROOT', os.getcwd())) / 'run/odin_faults')
            self.fault_recorder = FaultRecorder(directory / str(self.epoch) / 'fault.json',
                on_error=lambda error: self.get_logger().error('Fault snapshot write failed: ' + error))
            self.create_timer(.1, self.fault_recorder.poll)
        self.pub = self.create_publisher(Odometry, "/navigation/odometry", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/navigation/cloud", 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/navigation/sensor_pose", 10
        )
        self.health_pub = self.create_publisher(
            NavigationHealth, "/navigation/health", 10
        )
        self.tf = TransformBroadcaster(self)
        cloud_qos = QoSProfile(depth=5) if self.p["mode"] == "sim" else qos_profile_sensor_data
        self.create_subscription(Odometry, self.p["odom_topic"], self.odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.p["cloud_topic"], self.cloud, cloud_qos)
        self.create_timer(0.02, self.drain)
        self.create_timer(0.1, self.health)

    def raw_sample(self, m):
        def values(items):
            return [float(v) if math.isfinite(v) else str(v) for v in items]
        p, q, v, w = (m.pose.pose.position, m.pose.pose.orientation,
                      m.twist.twist.linear, m.twist.twist.angular)
        return {'stamp_ns': m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec,
                'observed_ros_ns': self.get_clock().now().nanoseconds,
                'observed_monotonic_ns': time.monotonic_ns(),
                'frame_id': m.header.frame_id, 'child_frame_id': m.child_frame_id,
                'position': values([p.x, p.y, p.z]),
                'quaternion_xyzw': values([q.x, q.y, q.z, q.w]),
                'linear_velocity': values([v.x, v.y, v.z]),
                'angular_velocity': values([w.x, w.y, w.z])}

    def latch_fault(self, reason, evidence):
        if self.fault:
            return
        previous_epoch = self.epoch
        self.epoch += 1
        self.fault = True
        self.first_fault_reason = self.reason = reason
        evidence = dict(evidence, reason=reason, previous_epoch=previous_epoch,
                        epoch=self.epoch, detected_ros_ns=self.get_clock().now().nanoseconds,
                        source=self.p['sensor'], odom_topic=self.p['odom_topic'],
                        cloud_topic=self.p['cloud_topic'],
                        world_quaternion=list(self.p['world_quaternion']),
                        body_to_source_translation=list(self.p['odom_sensor_translation']),
                        body_to_source_quaternion_xyzw=list(self.p['odom_sensor_quaternion']))
        if self.fault_recorder is not None:
            evidence['snapshot_path'] = str(self.fault_recorder.path)
            self.fault_recorder.trigger(evidence)
        message = json.dumps(evidence, ensure_ascii=False)
        self.fault_pub.publish(String(data=message))
        self.get_logger().error('First localization fault: ' + message)

    def destroy_node(self):
        if self.fault_recorder is not None:
            self.fault_recorder.close()
        return super().destroy_node()

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom(self, m):
        if self.fault_recorder is not None:
            self.fault_recorder.push(self.raw_sample(m))
        self.accept_odom(m)

    def drain(self):
        now = self.now_sec()
        while self.pending_cloud:
            m, arrival = self.pending_cloud[0]
            if (time.monotonic() - arrival > self.p["cloud_wait_timeout"]
                    or not fresh(now, self.stamp(m), self.p["cloud_timeout"])):
                self.pending_cloud.popleft()
                self.reason = "Cloud expired while waiting for odometry"
                continue
            if not self.publish_cloud(m):
                break
            self.pending_cloud.popleft()

    def accept_odom(self, m):
        t = self.stamp(m)
        now = self.now_sec()
        if not self.clock.update(now):
            self.latch_fault('clock discontinuity; restart required', {
                'raw_current': self.raw_sample(m), 'last_accepted_raw': self.last_accepted_raw})
            self.history.clear()
        if self.fault or not fresh(now, t, self.p["odom_timeout"]):
            return
        if (
            m.header.frame_id.lstrip("/") != self.p["source_frame"]
            or m.child_frame_id.lstrip("/") != self.p["source_child"]
        ):
            self.reason = "unexpected odometry frame"
            return
        if t <= self.last_odom:
            return
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        v = m.twist.twist.linear
        w = m.twist.twist.angular
        try:
            source_quat = [q.x, q.y, q.z, q.w]
            pos, quat, vel, omega = body_state(
                [p.x, p.y, p.z],
                [q.x, q.y, q.z, q.w],
                [v.x, v.y, v.z],
                [w.x, w.y, w.z],
                self.p["odom_sensor_translation"],
                self.p["odom_sensor_quaternion"],
                self.p["velocity_world"],
            )
            pos = self.world_rotation @ pos
            quat = quaternion(self.world_rotation @ rotation(quat))
            vel = self.world_rotation @ vel
            twist_covariance = np.array(m.twist.covariance).reshape(6, 6).copy()
            pose_covariance, twist_covariance = body_covariances(
                m.pose.covariance, twist_covariance.reshape(-1), source_quat,
                self.p["odom_sensor_translation"], self.p["odom_sensor_quaternion"],
                self.world_rotation, self.p["velocity_world"],
            )
        except ValueError:
            self.reason = "invalid odometry"
            return
        if self.history:
            dt = t - self.history[-1][0]
            angle = math.acos(
                float(
                    np.clip(
                        (np.trace(rotation(self.history[-1][2]).T @ rotation(quat)) - 1)
                        / 2,
                        -1,
                        1,
                    )
                )
            )
            distance = float(np.linalg.norm(pos - self.history[-1][1]))
            if distance > max(
                0.5, 4 * dt
            ) or angle > max(0.35, 4 * dt):
                self.latch_fault('localization position jump', {
                    'dt_s': dt, 'position_delta_m': distance,
                    'rotation_delta_rad': angle, 'rotation_delta_deg': math.degrees(angle),
                    'position_threshold_m': max(.5, 4*dt),
                    'rotation_threshold_rad': max(.35, 4*dt),
                    'position_triggered': distance > max(.5, 4*dt),
                    'rotation_triggered': angle > max(.35, 4*dt),
                    'previous_body_position': self.history[-1][1].tolist(),
                    'previous_body_quaternion_xyzw': self.history[-1][2].tolist(),
                    'current_body_position': pos.tolist(),
                    'current_body_quaternion_xyzw': quat.tolist(),
                    'last_accepted_raw': self.last_accepted_raw,
                    'raw_current': self.raw_sample(m)})
                return
        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (
            map(float, pos)
        )
        (
            out.pose.pose.orientation.x,
            out.pose.pose.orientation.y,
            out.pose.pose.orientation.z,
            out.pose.pose.orientation.w,
        ) = map(float, quat)
        vb = rotation(quat).T @ vel
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = (
            map(float, vb)
        )
        (
            out.twist.twist.angular.x,
            out.twist.twist.angular.y,
            out.twist.twist.angular.z,
        ) = map(float, omega)
        out.pose.covariance = pose_covariance
        out.twist.covariance = twist_covariance
        self.pub.publish(out)
        self.history.append((t, pos, quat))
        self.last_odom = t
        self.last_accepted_raw = self.raw_sample(m)
        self.arrival = time.monotonic()
        self.reason = "ok"
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = "base_link"
        (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ) = map(float, pos)
        tf.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(tf)

    def at(self, t):
        for a, b in zip(self.history, list(self.history)[1:]):
            if a[0] <= t <= b[0]:
                u = (t - a[0]) / (b[0] - a[0])
                qb = b[2] if np.dot(a[2], b[2]) >= 0 else -b[2]
                q = (1 - u) * a[2] + u * qb
                q /= np.linalg.norm(q)
                return (1 - u) * a[1] + u * b[1], rotation(q)
        if self.history and abs(t - self.history[-1][0]) < self.p["cloud_pose_tolerance"]:
            return self.history[-1][1], rotation(self.history[-1][2])
        raise ValueError("no pose at scan timestamp")

    def cloud(self, m):
        t = self.stamp(m)
        if m.header.frame_id.lstrip("/") != self.p["cloud_frame"]:
            self.reason = "Unexpected point cloud frame"
            return
        if self.fault or not fresh(self.now_sec(), t, self.p["cloud_timeout"]):
            return
        self.pending_cloud.append((m, time.monotonic()))
        self.drain()

    def publish_cloud(self, m):
        t = self.stamp(m)
        if self.fault or t <= self.last_cloud:
            return True
        try:
            pos, r = self.at(t)
        except ValueError:
            return False  # 等待乱序到达的里程计，禁止伪造时间或无限外推。
        try:
            points = cloud_xyz(m)
            sensor = pos + r @ np.array(self.p["sensor_translation"])
            if not self.p["cloud_in_world"]:
                points = points @ (r @ rotation(self.p["sensor_quaternion"])).T + sensor
            else:
                points = points @ self.world_rotation.T
            dist = np.linalg.norm(points - sensor, axis=1)
            points = points[
                (dist >= self.p["min_range"]) & (dist <= self.p["max_range"])
            ]
            # pos is base_link at the scan stamp, not the offset lidar origin.
            # Reject self returns before voxelization and map raycasting/inflation.
            if self.p["body_filter_radius"] > 0:
                points = points[
                    np.linalg.norm(points - pos, axis=1) > self.p["body_filter_radius"]
                ]
            # A well-formed, timestamped empty/no-return scan is a live sensor frame.
            # Publish it without inventing free-space observations; stale/malformed
            # input is still rejected above. One point per 10 cm voxel bounds CPU work.
            _, idx = np.unique(
                np.floor(points / 0.1).astype(np.int32), axis=0, return_index=True
            )
            points = points[idx]
            h = Header(stamp=m.header.stamp, frame_id="odom")
            pose = PoseStamped()
            pose.header = h
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
                float, sensor
            )
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(h, points.astype(np.float32))
            )
            self.last_cloud = t
        except (ValueError, AssertionError, TypeError) as e:
            self.reason = str(e)
        return True

    def health(self):
        now = self.now_sec()
        h = NavigationHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = "odom"
        h.epoch = self.epoch
        h.source = "gazebo_truth" if self.p["mode"] == "sim" else self.p["sensor"]
        unique = self.count_publishers(self.p["odom_topic"]) == 1
        unique_cloud = self.count_publishers(self.p["cloud_topic"]) == 1
        h.odometry_age = max(0.0, now - self.last_odom)
        h.cloud_age = max(0.0, now - self.last_cloud)
        correction_fresh = (not self.p["require_cloud_for_odometry"]
                            or (unique_cloud and fresh(now, self.last_cloud, self.p["cloud_timeout"])))
        h.odometry_valid = bool(
            not self.fault
            and self.p["calibrated"]
            and unique
            and fresh(now, self.last_odom, self.p["odom_timeout"])
            and time.monotonic() - self.arrival < 1.0
            and correction_fresh
        )
        h.cloud_valid = bool(
            not self.fault and unique_cloud and fresh(now, self.last_cloud, self.p["cloud_timeout"])
        )
        if not self.p["calibrated"]:
            self.reason = "Calibration unverified: diagnostic output only"
        elif (not fresh(now, self.last_odom, self.p["odom_timeout"])
              or time.monotonic() - self.arrival >= 1.0):
            self.reason = "Selected odometry missing/stale; check host time synchronization"
        elif not correction_fresh:
            self.reason = "Selected scan missing/stale; odometry alone is insufficient"
        if not unique_cloud:
            self.reason = "selected cloud source publisher count is not one"
        h.reason = self.first_fault_reason if self.fault and self.first_fault_reason else (
            self.reason
            if unique
            else "selected odometry source publisher count is not one"
        )
        self.health_pub.publish(h)


def main():
    rclpy.init()
    n = StateAdapter()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
