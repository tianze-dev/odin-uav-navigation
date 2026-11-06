"""Isolated MINCO trajectory sampler; no arming or PX4 writes."""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from traj_utils.msg import PolyTraj
from quadrotor_msgs.msg import PositionCommand
from uav_interfaces.msg import NavigationStatus, PlanSafety, TrajectoryExecution
from uav_state_adapter.core import Polynomial, fresh, yaw, ClockGuard
from uav_px4_bridge.yaw_profile import YawProfile


class TrajectoryServer(Node):
    def __init__(self, **node_options):
        super().__init__("traj_server", **node_options)
        self.limits = [self.declare_parameter('execution/max_' + key, default).value
                       for key, default in (('vel', 1.0), ('acc', 1.0), ('jer', 3.0))]
        if not all(math.isfinite(v) and v > 0 for v in self.limits):
            raise ValueError('Execution limits must be finite and positive')
        self.task = None
        self.safety = {}
        self.current = None
        self.pending = None
        self.highest_id = 0
        self.retired_id = 0
        self.execution_reason = "waiting for task"
        self.yaw = 0.0
        self.yaw_mode = self.declare_parameter(
            "execution/yaw_mode", "follow", ParameterDescriptor(read_only=True)
        ).value
        if self.yaw_mode not in ("hold", "follow"):
            raise ValueError("execution/yaw_mode must be hold or follow")
        self.measured_yaw = None
        self.held_yaw = None
        self.yaw_profile = YawProfile(self.declare_parameter("execution/yaw_max_acc", 0.).value)
        self.previous = None
        self.clock = ClockGuard()
        self.pub = self.create_publisher(PositionCommand, "/planning/position_cmd", 10)
        self.fault_pub = self.create_publisher(String, "/planning/executor_fault", 10)
        self.execution_pub = self.create_publisher(
            TrajectoryExecution, "/planning/execution_status", 1)
        self.create_subscription(PolyTraj, "/planning/trajectory", self.accept, 10)
        self.create_subscription(
            NavigationStatus, "/navigation/status", self.status, 10
        )
        self.create_subscription(PlanSafety, "/planning/safety", self.safety_cb, 10)
        self.create_subscription(Odometry, "/navigation/odometry", self.odom, 10)
        self.create_timer(0.02, self.tick)

    def safety_cb(self, m):
        if not self.task or m.session != self.task.session:
            return
        previous = self.safety.get(m.traj_id)
        timestamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if (not fresh(self.now(), timestamp, .3) or (previous is not None and
                timestamp < previous.header.stamp.sec + previous.header.stamp.nanosec * 1e-9)):
            return
        self.safety[m.traj_id] = m
        # 撤销先于轨迹消息到达也有效；当前已经接管的轨迹不能回滚成旧轨迹。
        if not m.valid and (self.current is None or m.traj_id != self.current[2].traj_id):
            self.retired_id = max(self.retired_id, m.traj_id)
            if self.pending and self.pending[2].traj_id <= self.retired_id:
                self.pending = None
            self.execution_reason = "pending trajectory revoked"
            self.execution_status()
        for k in sorted(self.safety)[:-8]:
            self.safety.pop(k)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def execution_status(self):
        m = TrajectoryExecution()
        m.header.stamp = self.get_clock().now().to_msg()
        m.session = self.task.session if self.task else 0
        m.current_traj_id = self.current[2].traj_id if self.current else 0
        m.pending_traj_id = self.pending[2].traj_id if self.pending else 0
        m.retired_traj_id = self.retired_id
        m.reason = self.execution_reason
        self.execution_pub.publish(m)

    def retire_pending(self, reason):
        self.get_logger().warning(f"trajectory {self.pending[2].traj_id}: {reason}")
        self.retired_id = max(self.retired_id, self.pending[2].traj_id)
        self.pending = None
        self.execution_reason = reason

    def authorized(self, m, now):
        return not self.authorization_reason(m, now)

    def authorization_reason(self, m, now):
        s = self.safety.get(m.traj_id)
        if s is None:
            return "safety authorization missing"
        if s.session != m.session:
            return "safety authorization session mismatch"
        if not s.valid:
            return "safety authorization revoked: " + (s.reason or "unspecified")
        if not fresh(now, s.header.stamp.sec + s.header.stamp.nanosec * 1e-9, .3):
            return "safety authorization timestamp stale/future"
        if now > s.valid_until.sec + s.valid_until.nanosec * 1e-9:
            return "safety authorization lease expired"
        return ""

    def blocked(self, reason):
        reason = "command blocked: " + reason
        if self.execution_reason != reason:
            self.execution_reason = reason
            self.get_logger().warning(reason)
            self.execution_status()

    def odom(self, m):
        q = m.pose.pose.orientation
        components = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(components).all() or np.linalg.norm(components) < 1e-6:
            return
        try:
            self.measured_yaw = yaw(components)
            if self.current is None:
                self.yaw = self.measured_yaw
                self.yaw_profile.reset(self.yaw)
            if self.task and self.task.control_requested and self.held_yaw is None:
                self.held_yaw = self.measured_yaw
        except ValueError:
            pass

    def status(self, m):
        if (
            self.task is None
            or self.task.session != m.session
            or (m.control_requested and not self.task.control_requested)
            or not m.control_requested
        ):
            self.current = self.pending = None
            self.safety.clear()
            self.highest_id = self.retired_id = 0
            self.execution_reason = "task reset"
            self.previous = None
            # 每次新接管记录实际朝向；重规划、暂停和恢复沿用同一航向。
            self.held_yaw = self.measured_yaw if m.control_requested else None
            if self.measured_yaw is not None:
                self.yaw = self.measured_yaw
                self.yaw_profile.reset(self.yaw)
        self.task = m
        self.execution_status()

    def reject(self, reason):
        self.fault_pub.publish(String(data=reason))
        self.get_logger().warning(reason)

    def accept(self, m):
        start = m.start_time.sec + m.start_time.nanosec * 1e-9
        now = self.now()
        if (
            not self.task
            or m.session != self.task.session
            or not self.task.control_requested
        ):
            return
        if m.traj_id <= max(self.highest_id, self.retired_id):
            return
        try:
            if m.order != 5 or start > now + 1.0:
                raise ValueError("trajectory time/order invalid")
            if start < now - .1:
                self.retired_id = max(self.retired_id, m.traj_id)
                self.execution_reason = "trajectory arrived after activation deadline"
                self.get_logger().warning(f"trajectory {m.traj_id}: {self.execution_reason}")
                self.execution_status()
                return
            traj = Polynomial(m.duration, m.coef_x, m.coef_y, m.coef_z, self.limits)
            if (
                not math.isfinite(m.valid_duration)
                or not 0 < m.valid_duration <= traj.duration + 0.001
            ):
                raise ValueError("invalid validated duration")
            if self.current:
                old = self.current[0].sample(start - self.current[1])
                new = traj.sample(0)
                if any(
                    np.linalg.norm(a - b) > tol
                    for a, b, tol in zip(old, new, [0.08, 0.2, 0.5])
                ):
                    raise ValueError("replacement PVA discontinuity")
            if self.pending:
                raise ValueError("pending trajectory must be resolved before replacement")
            self.pending = (traj, start, m)
            self.highest_id = m.traj_id
            self.execution_reason = "trajectory accepted; waiting for activation"
            self.execution_status()
        except ValueError as e:
            self.reject(str(e))

    def tick(self):
        now = self.now()
        if not self.clock.update(now):
            self.current = self.pending = None
            self.measured_yaw = self.held_yaw = None
            self.reject("executor clock reset")
            self.execution_reason = "executor clock reset"
            self.execution_status()
            return
        if not self.task or not self.task.control_requested:
            self.execution_status()
            return
        if self.pending and now >= self.pending[1]:
            candidate, start, message = self.pending
            if now > start + .1:
                self.retire_pending("activation deadline missed")
            elif self.authorized(message, now):
                continuous = self.current is None or all(
                    np.linalg.norm(a - b) <= tol for a, b, tol in zip(
                        self.current[0].sample(now - self.current[1]),
                        candidate.sample(now - start), [.08, .2, .5]))
                if continuous:
                    self.current = self.pending
                    self.pending = None
                    self.execution_reason = "trajectory active"
                else:
                    self.retire_pending("activation PVA discontinuity")
        self.execution_status()
        if not self.current:
            return
        traj, start, m = self.current
        # A newly pending trajectory's safety message must not authorize an older one.
        authorization_reason = self.authorization_reason(m, now)
        if authorization_reason:
            self.blocked(f"trajectory {m.traj_id}: {authorization_reason}")
            return
        t = now - start
        if t > traj.duration and np.linalg.norm(traj.sample(traj.duration)[1]) > 0.05:
            self.blocked(f"trajectory {m.traj_id}: moving trajectory duration elapsed")
            return
        p, v, a, j = traj.sample(t)
        dt = (
            0.02 if self.previous is None else max(0.001, min(0.1, now - self.previous))
        )
        self.previous = now
        if self.yaw_mode == "hold":
            if self.held_yaw is None:
                return  # 没有有效朝向时，不猜测为 0 度并意外转头。
            self.yaw, yaw_rate = self.held_yaw, 0.0
        else:
            look = traj.sample(min(t + 0.6, traj.duration))[0] - p
            target = (
                math.atan2(look[1], look[0]) if np.linalg.norm(look[:2]) > 0.1 else self.yaw
            )
            self.yaw, yaw_rate = self.yaw_profile.step(target, dt)
        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "odom"
        cmd.trajectory_id = m.traj_id
        cmd.trajectory_flag = 1
        cmd.position.x, cmd.position.y, cmd.position.z = map(float, p)
        cmd.velocity.x, cmd.velocity.y, cmd.velocity.z = map(float, v)
        cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z = map(float, a)
        cmd.jerk.x, cmd.jerk.y, cmd.jerk.z = map(float, j)
        cmd.yaw = self.yaw
        cmd.yaw_dot = yaw_rate
        self.pub.publish(cmd)
        if self.execution_reason.startswith("command blocked:"):
            self.execution_reason = "trajectory active"
            self.get_logger().info(f"trajectory {m.traj_id}: command stream resumed")
            self.execution_status()


def main():
    rclpy.init()
    n = TrajectoryServer()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
