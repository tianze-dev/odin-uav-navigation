"""PX4 position Offboard bridge; explicit SITL services or real RC takeover.
Odin native state is the sole external localization input. ROS uses ENU/FLU.
"""

from .pilot import PilotController
from collections import deque
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from quadrotor_msgs.msg import PositionCommand
from uav_interfaces.msg import NavigationHealth, TrajectoryExecution
from px4_msgs.msg import (
    VehicleOdometry,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint,
    EstimatorStatusFlags,
    FailsafeFlags,
)
from uav_state_adapter.core import (
    rotation,
    quaternion,
    px4_pose,
    px4_variances,
    yaw,
    wrap,
    fresh,
    ENU_NED,
    FLU_FRD,
    ClockGuard,
)


def xyz(v):
    return np.array([v.x, v.y, v.z], float)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class PX4Bridge(Node):
    def __init__(self, **node_options):
        super().__init__("uav_px4_bridge", **node_options)
        self.prefix = self.declare_parameter("px4_namespace", "/scut").value.rstrip("/")
        self.mode = self.declare_parameter("mode", "sim").value
        self.publish_control = self.declare_parameter("publish_control", self.mode == "sim").value
        self.publish_ev = self.declare_parameter("publish_external_odometry", self.mode == "sim").value
        self.ev_max_rate = self.declare_parameter("external_odometry_max_rate", 50.0).value
        if not math.isfinite(self.ev_max_rate) or not 1. <= self.ev_max_rate <= 200.:
            raise ValueError("external_odometry_max_rate must be between 1 and 200 Hz")
        self.entry_mode = self.declare_parameter("offboard_entry", "rc" if self.mode == "real" else "service").value
        self.require_land = self.declare_parameter("require_landed_state", self.mode == "sim").value
        self.entry_timeout = self.declare_parameter("offboard_entry_timeout", 60. if self.mode == "real" else 5.).value
        self.takeover_height = self.declare_parameter("minimum_takeover_height", .5).value
        if self.entry_mode not in ("rc", "service") or self.mode not in ("sim", "real", "replay"):
            raise ValueError("Invalid PX4 bridge mode/offboard_entry")
        if not all(math.isfinite(x) and x > 0 for x in (self.entry_timeout, self.takeover_height)):
            raise ValueError("Invalid bridge timing")
        if self.mode == "replay" and (self.publish_ev or self.publish_control):
            raise ValueError("Replay must not create PX4 writers")
        self.require_ev = self.declare_parameter(
            "require_external_position", True
        ).value
        self.phase = "WAITING"
        self.reason = "waiting for state"
        self.health = None
        self.nav = None
        self.local = None
        self.vehicle = None
        self.land = None
        self.ekf = None
        self.failsafe = None
        self.px4_odom = None
        self.state_times = {}
        self.state_walls = {}
        self.state_stamps = {}
        self.qpx4 = None
        self.alignment = None
        self.samples = deque(maxlen=100)
        self.cmd = None
        self.cmd_received_wall = None
        self.executor_state = None
        self.executor_timeout_snapshot = None
        self.previous_tick_wall = None
        self.epoch = None
        self.resets = None
        self.clock = ClockGuard()
        self.entered = 0.0
        self.command_time = 0.0
        self.hold = None
        self.nav_wall = 0.0
        self.last_ref_time = 0.0
        self.exit_reason = ""
        self.localization_fault = False
        self.control_fault = ""
        self.last_ev_time = 0.0
        self.last_ev_wall = 0.0
        self.last_ev_sample = 0
        self.ev_started = False
        self.ground_z = None
        self.last_alignment_stamp = None
        self.enable_time = 0.0
        self.return_mode = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.status_pub = self.create_publisher(
            String, "/navigation/control_status", 10
        )
        self.diag_pub = self.create_publisher(
            String, "/navigation/control_diagnostics", 10
        )
        self.ev = self.publisher("vehicle_visual_odometry", VehicleOdometry) if self.publish_ev else None
        self.sp = self.publisher("trajectory_setpoint", TrajectorySetpoint) if self.publish_control else None
        self.off = self.publisher("offboard_control_mode", OffboardControlMode) if self.publish_control else None
        self.commands = (self.publisher("vehicle_command", VehicleCommand)
                         if self.publish_control and self.entry_mode == "service" else None)
        for name, kind, attr in [
            ("vehicle_local_position", VehicleLocalPosition, "local"),
            ("vehicle_status", VehicleStatus, "vehicle"),
            ("vehicle_land_detected", VehicleLandDetected, "land"),
            ("estimator_status_flags", EstimatorStatusFlags, "ekf"),
            ("failsafe_flags", FailsafeFlags, "failsafe"),
        ]:
            self.create_subscription(
                kind,
                self.topic("out", name, kind),
                lambda m, a=attr: self.receive(a, m),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            VehicleOdometry,
            self.topic("out", "vehicle_odometry", VehicleOdometry),
            self.px4odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleCommandAck,
            self.topic("out", "vehicle_command_ack", VehicleCommandAck),
            self.ack,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/navigation/odometry", self.nav_cb, 10)
        self.create_subscription(
            NavigationHealth, "/navigation/health", self.health_cb, 10
        )
        self.create_subscription(
            PositionCommand,
            "/planning/position_cmd",
            self.command_cb,
            10,
        )
        self.create_subscription(TrajectoryExecution, "/planning/execution_status",
                                 lambda m: setattr(self, "executor_state", m), 1)
        self.create_subscription(
            String,
            "/planning/executor_fault",
            lambda m: self.exit("Executor: " + m.data),
            10,
        )
        self.create_service(SetBool, "/navigation/control_enable", self.enable)
        self.create_service(Trigger, "/navigation/reset_control_fault", self.reset_fault)
        self.pilot = PilotController(self)
        self.create_timer(0.02, self.tick)
        if self.mode == "real" and self.publish_ev:
            self.create_timer(1. / self.ev_max_rate, self.publish_latest_ev)

    def topic(self, direction, name, kind):
        ver = getattr(kind, "MESSAGE_VERSION", 0)
        return f"{self.prefix}/fmu/{direction}/{name}" + (f"_v{ver}" if ver else "")

    def publisher(self, name, kind):
        return self.create_publisher(
            kind, self.topic("in", name, kind), qos_profile_sensor_data
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive(self, attr, m):
        stamp_us = int(m.timestamp)
        if stamp_us <= self.state_stamps.get(attr, 0):
            return  # 重发旧 PX4 状态不能刷新心跳。
        self.state_stamps[attr] = stamp_us
        setattr(self, attr, m)
        self.state_times[attr] = self.now()
        self.state_walls[attr] = time.monotonic()
        if attr == "local":
            counters = (
                m.xy_reset_counter,
                m.z_reset_counter,
                m.heading_reset_counter,
                m.vxy_reset_counter,
                m.vz_reset_counter,
            )
            if self.resets is not None and counters != self.resets:
                if self.armed():
                    self.invalidate_control("PX4 estimator reset")
                else:
                    self.samples.clear()
                    self.alignment = None
                    self.ground_z = None
            self.resets = counters

    def px4odom(self, m):
        if (m.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or int(m.timestamp) <= self.state_stamps.get("q", 0)
                or not np.isfinite(np.r_[m.q, m.position, m.velocity]).all()):
            return
        try:
            self.qpx4 = quaternion(
                ENU_NED @ rotation([m.q[1], m.q[2], m.q[3], m.q[0]]) @ FLU_FRD
            )
            self.state_times["q"] = self.now()
            self.state_walls["q"] = time.monotonic()
            self.state_stamps["q"] = int(m.timestamp)
            self.px4_odom = m
        except ValueError:
            pass

    def ack(self, m):
        self.pilot.ack(m)
        if (
            m.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            and m.result not in [0, 5]
            and self.phase == "PREFILL"
        ):
            self.exit("PX4 rejected Offboard mode")

    def health_cb(self, m):
        if self.epoch is not None and m.epoch != self.epoch:
            if self.armed():
                self.localization_fault = True
                self.invalidate_control("Localization epoch changed")
            else:
                self.exit("Localization epoch changed")
            self.alignment = None
            self.samples.clear()
        self.epoch = m.epoch
        self.health = m

    def command_cb(self, m):
        self.cmd = m
        self.cmd_received_wall = time.monotonic()

    def executor_timeout_details(self, now, tick_gap):
        state = self.executor_state
        return {
            "stamp": now,
            "command_age_s": now - stamp(self.cmd) if self.cmd else None,
            "command_receive_age_s": (time.monotonic() - self.cmd_received_wall
                                      if self.cmd_received_wall is not None else None),
            "command_frame": self.cmd.header.frame_id if self.cmd else None,
            "command_traj_id": self.cmd.trajectory_id if self.cmd else None,
            "bridge_tick_gap_s": tick_gap,
            "executor_age_s": now - stamp(state) if state else None,
            "executor_session": state.session if state else None,
            "executor_current_id": state.current_traj_id if state else None,
            "executor_pending_id": state.pending_traj_id if state else None,
            "executor_retired_id": state.retired_traj_id if state else None,
            "executor_reason": state.reason if state else "executor status unavailable",
        }

    def nav_cb(self, m):
        self.nav = m
        self.nav_wall = time.monotonic()
        if self.publish_ev and self.mode != "real":
            self.send_external_odometry(m)

    def publish_latest_ev(self):
        if self.nav is not None:
            self.send_external_odometry(self.nav)

    def send_external_odometry(self, m):
        if (
            not self.publish_ev
            or self.localization_fault
            or self.mode == "replay"
            or not self.source_ready()
        ):
            return
        sample_us = m.header.stamp.sec * 1000000 + m.header.stamp.nanosec // 1000
        if self.mode == "real" and sample_us <= self.last_ev_sample:
            return
        # The first native EV stream must start with a fresh DISARMED status.
        # Once established it continues during flight; silence is never proof of DISARMED.
        if self.mode == "real" and not self.ev_started and (
                not self.state_fresh("vehicle", 2.) or self.armed()):
            return
        if (
            self.count_publishers("/navigation/odometry") != 1
            or self.count_publishers("/navigation/health") != 1
            or self.count_publishers(
                self.topic("in", "vehicle_visual_odometry", VehicleOdometry)
            )
            != 1
        ):
            self.exit("Multiple external odometry writers")
            return
        q = m.pose.pose.orientation
        try:
            p, qn = px4_pose(xyz(m.pose.pose.position), [q.x, q.y, q.z, q.w])
            vn = ENU_NED @ rotation([q.x, q.y, q.z, q.w]) @ xyz(m.twist.twist.linear)
            variances = (px4_variances(m.pose.covariance, m.twist.covariance,
                                      [q.x, q.y, q.z, q.w]) if self.mode == "real" else None)
        except ValueError:
            self.exit("Invalid external odometry geometry or covariance")
            return
        if (
            self.alignment is not None
            and self.land is not None
            and not self.land.landed
            and self.px4_state_ready()
        ):
            r, t, angle = self.alignment
            discrepancy = np.linalg.norm(
                r @ xyz(m.pose.pose.position)
                + t
                - ENU_NED @ np.array([self.local.x, self.local.y, self.local.z])
            )
            heading_error = (
                abs(wrap(yaw(self.qpx4) - yaw([q.x, q.y, q.z, q.w]) - angle))
                if self.qpx4 is not None
                else 0.0
            )
            if discrepancy > 0.5 or heading_error > 0.2:
                self.invalidate_control("Localization/PX4 frame disagreement")
                # 对齐失效只撤销导航；同 epoch、有效且连续的定位仍交给 PX4。
        out = VehicleOdometry()
        out.timestamp = int(self.now() * 1e6)
        out.timestamp_sample = sample_us
        out.pose_frame = (
            VehicleOdometry.POSE_FRAME_NED
            if self.mode == "sim"
            else VehicleOdometry.POSE_FRAME_FRD
        )
        out.velocity_frame = (
            VehicleOdometry.VELOCITY_FRAME_NED
            if self.mode == "sim"
            else VehicleOdometry.VELOCITY_FRAME_FRD
        )
        out.position = list(map(float, p))
        out.q = [float(qn[3]), *map(float, qn[:3])]
        out.velocity = list(map(float, vn))
        out.angular_velocity = list(map(float, FLU_FRD @ xyz(m.twist.twist.angular)))
        if variances is None:
            out.position_variance = [0.0025] * 3
            out.orientation_variance = [0.0025] * 3
            out.velocity_variance = [0.01] * 3
        else:
            out.position_variance, out.orientation_variance, out.velocity_variance = variances
        out.reset_counter = int(self.epoch or 0) % 256
        out.quality = 100 if self.mode == "sim" else 0
        self.ev.publish(out)
        self.last_ev_sample = sample_us
        self.ev_started = True
        self.last_ev_time = self.now()
        self.last_ev_wall = time.monotonic()





    def armed(self):
        return bool(self.vehicle and self.vehicle.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def state_fresh(self, key, limit):
        return (fresh(self.now(), self.state_times.get(key, 0.), limit)
                and time.monotonic() - self.state_walls.get(key, 0.) < limit)

    def ground_ready(self):
        if not self.state_fresh("vehicle", 2.) or self.armed():
            return False
        if self.require_land or self.state_fresh("land", 2.):
            return bool(self.state_fresh("land", 2.) and self.land.landed)
        # PX4 1.15 的现有固件未导出 land_detected。仅上锁静止对齐，不伪造着陆消息。
        return True

    def airborne_ready(self):
        if not self.armed():
            return False
        if self.require_land or self.state_fresh("land", 2.):
            return bool(self.state_fresh("land", 2.) and not self.land.landed)
        return bool(self.ground_z is not None and self.local is not None
                    and self.ground_z - self.local.z >= self.takeover_height)

    def real_guard_error(self, require_rc=True):
        if self.mode != "real":
            return ""
        if self.vehicle is None or self.vehicle.failsafe:
            return "PX4 failsafe or status unavailable"
        if not self.state_fresh("ekf", 2.) or not (
                self.ekf.cs_tilt_align and self.ekf.cs_ev_pos and self.ekf.cs_ev_hgt
                and self.ekf.cs_ev_yaw):
            return "Need fresh external position/height/yaw fusion"
        if (self.ekf.cs_ev_yaw_fault or self.ekf.reject_yaw
                or self.ekf.cs_inertial_dead_reckoning or self.ekf.cs_fake_pos
                or self.ekf.cs_fake_hgt
                or any(getattr(self.ekf, key) for key in self.ekf.get_fields_and_field_types()
                       if key.startswith("fs_bad_"))):
            return "PX4 estimator fusion fault"
        if require_rc and (not self.state_fresh("failsafe", 2.)
                           or self.failsafe.manual_control_signal_lost):
            return "RC takeover signal unavailable"
        if require_rc and self.phase == "ACTIVE":
            if (self.px4_odom is None or not np.isfinite(self.px4_odom.angular_velocity).all()
                    or np.linalg.norm(self.px4_odom.angular_velocity) > 1.5):
                return "Body angular rate exceeds the supported envelope"
        return ""

    def source_ready(self):
        """判断选定定位源；实机适配器同时要求有效的雷达校正流。"""
        if not (
            self.count_publishers("/navigation/odometry") == 1
            and self.count_publishers("/navigation/health") == 1
            and self.health is not None
            and fresh(self.now(), stamp(self.health), 0.3)
            and self.health.odometry_valid
            and self.nav is not None
            and self.nav.header.frame_id == "odom"
            and self.nav.child_frame_id == "base_link"
            and fresh(self.now(), stamp(self.nav), 0.25)
            and time.monotonic() - self.nav_wall < 0.5
        ):
            return False
        q = self.nav.pose.pose.orientation
        return bool(np.isfinite(np.r_[xyz(self.nav.pose.pose.position),
            xyz(self.nav.twist.twist.linear), xyz(self.nav.twist.twist.angular),
            q.x, q.y, q.z, q.w]).all() and np.linalg.norm([q.x, q.y, q.z, q.w]) > 1e-9)

    def px4_state_ready(self):
        return bool(
            self.local is not None and self.qpx4 is not None
            and self.vehicle is not None
            and all(self.state_fresh(k, limit)
                    for k, limit in [("local", .3), ("q", .3), ("vehicle", 2.)])
            and (not self.require_land or self.state_fresh("land", 2.))
            and self.local.xy_valid
            and self.local.z_valid
            and self.local.v_xy_valid and self.local.v_z_valid
            and np.isfinite([self.local.x, self.local.y, self.local.z,
                             self.local.vx, self.local.vy, self.local.vz]).all()
        )

    def position_healthy(self):
        return bool(not self.localization_fault and self.source_ready() and self.px4_state_ready()
                    and self.count_publishers(self.topic(
                        "in", "vehicle_visual_odometry", VehicleOdometry)) == 1
                    and not self.real_guard_error(require_rc=False))

    def healthy(self):
        return bool(not self.control_fault and self.phase != "FAULT"
                    and self.position_healthy() and self.health.cloud_valid
                    and not self.real_guard_error())

    def blocking_reason(self):
        if self.control_fault or self.localization_fault:
            return self.control_fault or "Localization fault latched"
        if not self.source_ready():
            return self.health.reason if self.health else "Waiting for selected localization"
        if not self.px4_state_ready():
            return "Waiting for valid, fresh PX4 position/velocity/attitude"
        if self.count_publishers(self.topic("in", "vehicle_visual_odometry", VehicleOdometry)) != 1:
            return "Need one external odometry writer"
        if not self.health.cloud_valid:
            return "Registered obstacle cloud missing/stale"
        return self.real_guard_error()

    def invalidate_control(self, why):
        if not self.control_fault:
            self.control_fault = why
        self.exit(why)
        self.alignment = None
        self.samples.clear()

    def reset_rejection(self):
        if self.mode == "replay":
            return "回放模式不允许控制复位"
        if not (self.control_fault or self.localization_fault or self.phase == "FAULT"):
            return "没有需要复位的控制桥故障"
        # 恢复外部定位前，PX4 的位置有效位可能尚未恢复；不能形成循环依赖。
        if not self.ground_ready():
            return "需要新鲜的 PX4 解锁/着陆状态，并已着陆、上锁"
        # PX4 上锁后可能保留 OFFBOARD 模式编号；以已上锁及桥接接管状态为准。
        if self.phase in ("ACTIVE", "PREFILL", "EXITING") or self.pilot.busy:
            return "请先结束控制接管和飞手操作"
        if (not self.source_ready() or self.count_publishers(
                self.topic("in", "vehicle_visual_odometry", VehicleOdometry)) != 1):
            return "需要唯一、有效且新鲜的定位源；请先处理状态适配器故障"
        px4_velocity_valid = (self.local is not None
            and fresh(self.now(), self.state_times.get("local", 0.), .3)
            and self.local.v_xy_valid and self.local.v_z_valid)
        if (np.linalg.norm(xyz(self.nav.twist.twist.linear)) > .1
                or (px4_velocity_valid and (
                    not np.isfinite([self.local.vx, self.local.vy, self.local.vz]).all()
                    or np.linalg.norm([self.local.vx, self.local.vy, self.local.vz]) > .1))):
            return "请等待机体在地面静止"
        return ""

    def reset_fault(self, _request, response):
        rejection = self.reset_rejection()
        if rejection:
            response.message = rejection
            return response
        self.localization_fault = False
        self.control_fault = ""
        self.alignment = None
        self.ground_z = None
        self.last_alignment_stamp = None
        self.samples.clear()
        self.cmd = self.hold = None
        self.phase = "WAITING"
        self.reason = "Fault reset on ground; waiting for fresh alignment"
        self.exit_reason = ""
        response.success = True
        response.message = "控制桥故障已复位，等待重新对齐；旧任务不会自动恢复"
        return response

    def command(self, main_mode):
        if not self.vehicle or self.mode == "replay" or self.commands is None:
            return
        m = VehicleCommand()
        m.timestamp = int(self.now() * 1e6)
        m.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        m.param1 = 1.0
        m.param2 = float(main_mode)
        m.param3 = 3.0 if main_mode == 4 else 0.0
        m.target_system = self.vehicle.system_id
        m.target_component = 1
        m.source_system = 250
        m.source_component = 191
        m.from_external = True
        self.commands.publish(m)
        self.command_time = self.now()

    def exit(self, why):
        self.reason = why
        if self.phase in ["ACTIVE", "PREFILL"]:
            self.phase = "EXITING"
            self.entered = self.now()
            self.exit_reason = why
            if self.local:
                self.hold = np.array([self.local.x, self.local.y, self.local.z])
            self.command(
                4
                if self.return_mode == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                else 3
            )
        else:
            self.reason = why
        self.cmd = None

    def finish_confirmed_release(self):
        # A delayed PX4 mode acknowledgement is recoverable. A genuine control
        # or localization fault still requires the explicit ground reset.
        if (self.phase in ("EXITING", "FAULT")
                and getattr(self, "exit_reason", "")
                and not self.control_fault and not self.localization_fault
                and self.vehicle is not None
                and self.state_fresh("vehicle", 2.0)
                and self.vehicle.nav_state in (
                    VehicleStatus.NAVIGATION_STATE_POSCTL,
                    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)):
            self.phase = "WAITING"
            self.reason = self.exit_reason
            self.cmd = None

    def enable(self, q, r):
        if not q.data:
            if self.phase in ["ACTIVE", "PREFILL"]:
                self.exit("Explicit release")
            self.cmd = None
            r.success = True
            r.message = "Releasing Offboard"
            return r
        if not self.publish_control:
            r.message = "只读观察模式；需重启并显式设置 enable_control:=true"
            return r
        if self.pilot.busy:
            r.success = False
            r.message = "飞手操作尚未完成"
            return r
        self.finish_confirmed_release()
        if self.control_fault or self.localization_fault or self.phase == "FAULT":
            r.message = "控制桥故障未复位；需要着陆、上锁后显式复位"
            return r
        allowed = [
            VehicleStatus.NAVIGATION_STATE_POSCTL,
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
        ]
        valid = (
            self.mode != "replay"
            and self.healthy()
            and self.alignment is not None
            and self.airborne_ready()
            and self.vehicle.nav_state in allowed
        )
        if self.phase == "ACTIVE":
            valid = True
        if self.require_ev:
            valid = (
                valid
                and self.ekf is not None
                and self.ekf.cs_ev_pos
                and self.now() - self.state_times.get("ekf", -1e9) < 2
            )
        if not valid:
            r.success = False
            r.message = (f"Rejected: {self.blocking_reason()}; aligned={self.alignment is not None}, "
                         f"armed={self.armed()}, airborne_ready={self.airborne_ready()}")
            return r
        self.entered = self.now()
        self.hold = np.array([self.local.x, self.local.y, self.local.z])
        self.cmd = None
        if self.phase != "ACTIVE":
            self.return_mode = self.vehicle.nav_state
            self.phase = "PREFILL"
            self.entered = self.now()
            self.hold = np.array([self.local.x, self.local.y, self.local.z])
            self.cmd = None
            self.enable_time = self.now()
        r.success = True
        r.message = ("预发悬停参考，等待遥控器切入 Offboard" if self.entry_mode == "rc"
                     else "Streaming hold references before Offboard")
        return r

    def publish(self, p, v=None, a=None, y=None, yr=0.0):
        if not self.publish_control or self.sp is None or self.off is None:
            return
        m = TrajectorySetpoint()
        m.timestamp = int(self.now() * 1e6)
        m.position = list(map(float, p))
        m.velocity = list(map(float, v if v is not None else [0.0, 0.0, 0.0]))
        m.acceleration = list(map(float, a if a is not None else [0.0, 0.0, 0.0]))
        m.jerk = [float("nan")] * 3
        m.yaw = float(self.local.heading if y is None else y)
        m.yawspeed = float(yr)
        o = OffboardControlMode()
        o.timestamp = m.timestamp
        o.position = True
        self.off.publish(o)
        self.sp.publish(m)

    def align(self):
        if (
            not self.position_healthy()
            or not self.ground_ready()
        ):
            return
        if (np.linalg.norm([self.local.vx, self.local.vy, self.local.vz]) > 0.1
                or np.linalg.norm(xyz(self.nav.twist.twist.linear)) > .1
                or (self.mode == "real" and (self.px4_odom is None
                    or not np.isfinite(self.px4_odom.angular_velocity).all()
                    or np.linalg.norm(self.px4_odom.angular_velocity) > .15))):
            self.samples.clear()
            return
        source_stamp = stamp(self.nav)
        if self.mode == "real" and source_stamp == self.last_alignment_stamp:
            return
        self.last_alignment_stamp = source_stamp
        q = self.nav.pose.pose.orientation
        qn = [q.x, q.y, q.z, q.w]
        ang = wrap(yaw(self.qpx4) - yaw(qn))
        c, s = math.cos(ang), math.sin(ang)
        r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        if self.mode == "real":
            tilt_error = math.acos(float(np.clip(
                (np.trace(rotation(self.qpx4).T @ r @ rotation(qn)) - 1) / 2, -1., 1.)))
            if tilt_error > math.radians(5):
                self.samples.clear()
                return
        offset = ENU_NED @ np.array(
            [self.local.x, self.local.y, self.local.z]
        ) - r @ xyz(self.nav.pose.pose.position)
        self.samples.append((self.now(), ang, offset))
        if len(self.samples) < (20 if self.mode == "real" else 40) or self.samples[-1][0] - self.samples[0][0] < 1.5:
            return
        angles = np.unwrap([x[1] for x in self.samples])
        offsets = np.array([x[2] for x in self.samples])
        if np.std(angles) < 0.03 and np.max(np.std(offsets, axis=0)) < 0.03:
            a = float(np.mean(angles))
            c, s = math.cos(a), math.sin(a)
            self.alignment = (
                np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]),
                offsets.mean(axis=0),
                a,
            )
            self.ground_z = float(self.local.z)

    def check_alignment(self):
        if self.alignment is None or not self.armed() or not self.px4_state_ready() or self.nav is None:
            return
        r, t, angle = self.alignment
        q = self.nav.pose.pose.orientation
        discrepancy = np.linalg.norm(r @ xyz(self.nav.pose.pose.position) + t
            - ENU_NED @ np.array([self.local.x, self.local.y, self.local.z]))
        heading_error = abs(wrap(yaw(self.qpx4) - yaw([q.x, q.y, q.z, q.w]) - angle))
        if discrepancy > .5 or heading_error > .2:
            self.invalidate_control("Localization/PX4 frame disagreement")

    def tick(self):
        now = self.now()
        tick_wall = time.monotonic()
        tick_gap = tick_wall - self.previous_tick_wall if self.previous_tick_wall is not None else None
        self.previous_tick_wall = tick_wall
        if not self.clock.update(now):
            self.localization_fault = True
            self.control_fault = self.control_fault or "Clock discontinuity"
            self.phase = "FAULT"
            self.reason = "clock discontinuity"
            self.alignment = None
            self.samples.clear()
            self.cmd = None
        self.align()
        self.check_alignment()
        self.pilot.tick()
        self.finish_confirmed_release()
        if self.phase not in ["ACTIVE", "PREFILL", "EXITING", "FAULT"]:
            available = (self.healthy()
                and self.alignment is not None
                and self.airborne_ready()
                and self.vehicle.nav_state
                in [
                    VehicleStatus.NAVIGATION_STATE_POSCTL,
                    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                ])
            self.phase = "OBSERVING" if not self.publish_control else ("AVAILABLE" if available else "WAITING")
            self.reason = self.blocking_reason() or (
                ("External odometry active; navigation control disabled" if self.publish_ev
                 else "Observe only; PX4 writers disabled") if not self.publish_control
                else "Waiting for alignment and airborne RC takeover")
        if self.phase in ["ACTIVE", "PREFILL"] and not self.healthy():
            self.exit("Source or PX4 state stale")
        if self.phase in ["ACTIVE", "PREFILL"] and (
            self.count_publishers(
                self.topic("in", "trajectory_setpoint", TrajectorySetpoint)
            )
            > 1
            or self.count_publishers("/planning/position_cmd") > 1
            or self.count_publishers(self.topic("in", "offboard_control_mode", OffboardControlMode)) > 1
        ):
            self.exit("Multiple control writers")
        if self.phase == "PREFILL":
            self.publish(self.hold)
            if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.phase = "ACTIVE"
                self.entered = now
            elif (self.entry_mode == "service" and now - self.entered > 1.0
                  and now - self.command_time > 0.5):
                self.command(6)
            if now - self.entered > self.entry_timeout:
                self.exit("Offboard entry timed out")
        elif self.phase == "ACTIVE":
            if self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.phase = "WAITING"
                self.reason = "pilot/PX4 takeover"
                self.cmd = None
            elif (
                self.cmd
                and fresh(now, stamp(self.cmd), 0.15)
                and self.cmd.header.frame_id == "odom"
            ):
                r, t, angle = self.alignment
                c = self.cmd
                pos = xyz(c.position)
                vel = xyz(c.velocity)
                acc = xyz(c.acceleration)
                predicted = r @ xyz(self.nav.pose.pose.position) + t
                px4pos = ENU_NED @ np.array([self.local.x, self.local.y, self.local.z])
                if not np.isfinite(np.r_[pos, vel, acc, c.yaw, c.yaw_dot]).all():
                    self.exit("Non-finite trajectory reference")
                elif np.linalg.norm(pos - xyz(self.nav.pose.pose.position)) > 1.5:
                    self.exit("Trajectory tracking error exceeds 1.5 m")
                elif np.linalg.norm(predicted - px4pos) > 0.5:
                    self.invalidate_control("Localization/PX4 frame disagreement")
                else:
                    self.publish(
                        ENU_NED @ (r @ pos + t),
                        ENU_NED @ r @ vel,
                        ENU_NED @ r @ acc,
                        wrap(math.pi / 2 - c.yaw - angle),
                        -c.yaw_dot,
                    )
            elif now - self.entered < 1.0:
                self.publish(self.hold)
            else:
                self.executor_timeout_snapshot = self.executor_timeout_details(now, tick_gap)
                import json
                self.get_logger().error("Trajectory executor expired: " +
                                        json.dumps(self.executor_timeout_snapshot))
                self.exit("Trajectory executor expired")
        elif self.phase == "EXITING":
            if (
                self.vehicle
                and self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.phase = "WAITING"
                self.reason = self.exit_reason
            elif (
                self.entry_mode == "service"
                and self.publish_control
                and now - self.entered < 0.5
                and self.local
                and self.local.xy_valid
                and self.local.z_valid
            ):
                self.publish(self.hold)
            elif now - self.entered > 1.0:
                self.phase = "FAULT"
                self.reason = self.exit_reason
        self.status_pub.publish(String(data=self.phase))
        import json

        self.diag_pub.publish(
            String(
                data=json.dumps(
                    {
                        "phase": self.phase,
                        "reason": self.reason,
                        "aligned": self.alignment is not None,
                        "healthy": bool(self.healthy()),
                        "position_healthy": self.position_healthy(),
                        "control_fault": self.control_fault,
                        "executor_timeout": self.executor_timeout_snapshot,
                        "localization_fault": self.localization_fault,
                        "external_odometry_active": bool(
                            (self.publish_ev and not self.localization_fault and self.source_ready())
                            and fresh(now, self.last_ev_time, .25)
                            and time.monotonic() - self.last_ev_wall < .5),
                        "publish_control": self.publish_control,
                        "publish_external_odometry": self.publish_ev,
                        "offboard_entry": self.entry_mode,
                        "blocking_reason": self.blocking_reason(),
                        "ground_reference_z": self.ground_z,
                        "airborne_ready": self.airborne_ready(),
                        "rc_available": bool(self.state_fresh("failsafe", 2.)
                                             and not self.failsafe.manual_control_signal_lost),
                        "reset_ready": not self.reset_rejection(),
                        "samples": len(self.samples),
                        "span": self.samples[-1][0] - self.samples[0][0]
                        if self.samples
                        else 0.0,
                        "ev_pos": bool(self.ekf and self.ekf.cs_ev_pos),
                        "source": self.health.source if self.health else None,
                        "armed": bool(self.vehicle and self.vehicle.arming_state == VehicleStatus.ARMING_STATE_ARMED),
                        "landed": self.land.landed if self.land else None,
                        "px4_mode": self.vehicle.nav_state if self.vehicle else None,
                        "px4_connected": bool(self.vehicle and now - self.state_times.get("vehicle", -1e9) < 2),
                        "pilot_enabled": bool(self.pilot.enabled and self.mode == "sim"),
                        "pilot_ready": self.pilot.ready(),
                        "pilot_busy": self.pilot.busy,
                        "pilot_state": self.pilot.state,
                        "pilot_message": self.pilot.message,
                        "command_ack": self.pilot.last_ack,
                    }
                )
            )
        )


def main():
    rclpy.init()
    n = PX4Bridge()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
