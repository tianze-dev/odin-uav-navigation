"""显式 SITL 飞手操作。命令由唯一 PX4 bridge 发布，构造函数不发命令。"""
import math
import time
from px4_msgs.msg import VehicleCommand, VehicleStatus
from uav_interfaces.srv import PilotCommand
from std_srvs.srv import Trigger


class PilotController:
    def __init__(self, bridge):
        self.b = bridge
        self.enabled = bridge.declare_parameter('manual_control_enabled', False).value
        self.state = 'IDLE'
        self.message = '等待显式操作'
        self.operation = ''
        self.started = 0.0
        self.origin_ready = False
        self.pending = set()
        self.last_ack = ''
        self.height = 2.5
        self.cancel_client = bridge.create_client(Trigger, '/navigation/cancel')
        self.service = bridge.create_service(PilotCommand, '/navigation/pilot_command', self.request)

    @property
    def busy(self):
        return self.state in ('PREPARING', 'ARMING', 'TAKING_OFF', 'LANDING', 'HOLDING', 'DISARMING')

    def ready(self):
        b = self.b
        return bool(self.enabled and b.publish_control and b.commands is not None and b.mode == 'sim' and b.health and
                    b.health.source == 'gazebo_truth' and b.healthy() and
                    b.alignment is not None and b.ekf and b.ekf.cs_ev_pos)

    def send(self, command, **params):
        b = self.b
        m = VehicleCommand()
        m.timestamp = int(b.now() * 1e6)
        m.command = command
        m.target_system = b.vehicle.system_id
        m.target_component = 1
        m.source_system = 250
        m.source_component = 191
        m.from_external = True
        for key, value in params.items():
            setattr(m, key, float(value))
        self.pending.add(command)
        b.commands.publish(m)

    def request(self, q, r):
        b = self.b
        if q.command not in ('arm', 'disarm', 'takeoff', 'hold', 'land'):
            r.message = '不支持的飞手操作'
            return r
        if not self.enabled or b.mode != 'sim' or not b.health or b.health.source != 'gazebo_truth':
            r.message = '本面板飞手操作仅在显式启用的 Gazebo SITL 中开放'
            return r
        if not b.vehicle or not b.land or b.now() - b.state_times.get('vehicle', -1e9) > 2 or b.now() - b.state_times.get('land', -1e9) > 2:
            r.message = '飞控状态过期'
            return r
        armed = b.vehicle.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.busy and q.command not in ('hold', 'land'):
            r.message = '上一条飞手操作尚未完成'
            return r
        if q.command in ('arm', 'takeoff'):
            if not self.ready() or not b.land.landed or b.phase in ('ACTIVE', 'PREFILL', 'EXITING'):
                r.message = '需要地面、健康定位、外部定位融合和坐标对齐'
                return r
            if q.command == 'arm' and armed:
                r.message = '已经解锁'
                return r
            if not math.isfinite(q.height) or not .5 <= q.height <= 4.0:
                r.message = '仿真起飞高度必须为 0.5–4.0 m'
                return r
            if armed and not self.origin_ready:
                r.message = '请先在地面上锁，再通过面板准备并起飞'
                return r
        if q.command == 'disarm' and not b.land.landed:
            r.message = '空中不允许上锁'
            return r
        if q.command in ('land', 'hold') and (not armed or b.land.landed):
            r.message = '需要已解锁且离地'
            return r
        if q.command == 'hold' and not b.position_healthy():
            r.message = '定位不健康，无法请求位置保持'
            return r
        self.operation = q.command
        self.started = time.monotonic()
        self.pending.clear()
        self.last_ack = ''
        self.height = q.height
        if q.command in ('arm', 'takeoff'):
            self.ground_z = b.local.z
            if armed:
                self.takeoff()
            else:
                self.state = 'PREPARING'
                self.message = '建立仿真地理原点并等待坐标对齐'
                self.send(VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
                          param5=47.397742, param6=8.545594, param7=488.0)
        elif q.command == 'disarm':
            self.state = 'DISARMING'
            self.message = '请求地面上锁'
            self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0)
        else:
            if self.cancel_client.service_is_ready():
                self.cancel_client.call_async(Trigger.Request())
            # 先撤销 bridge 的轨迹输出，避免飞手操作后旧轨迹重新夺回控制。
            b.cmd = None
            b.phase = 'WAITING'
            b.reason = '面板飞手接管：' + q.command
            self.state = 'LANDING' if q.command == 'land' else 'HOLDING'
            self.message = '请求降落' if q.command == 'land' else '请求悬停'
            if q.command == 'land':
                self.send(VehicleCommand.VEHICLE_CMD_NAV_LAND, param4=float('nan'),
                          param5=float('nan'), param6=float('nan'), param7=float('nan'))
            else:
                self.send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1, param2=4, param3=3)
        r.success = True
        r.message = '请求已接受；以飞控状态及命令应答为准：' + self.message
        return r

    def takeoff(self):
        self.ground_z = self.b.local.z
        self.state = 'TAKING_OFF'
        self.message = f'起飞中，目标上升 {self.height:.1f} m'
        self.send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param4=float('nan'),
                  param5=float('nan'), param6=float('nan'),
                  param7=float(self.b.local.ref_alt) - self.ground_z + self.height)
        if self.b.vehicle.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1)

    def ack(self, m):
        if m.command not in self.pending:
            return
        self.last_ack = f'command={m.command}, result={m.result}'
        if m.result not in (0, 5):
            self.state = 'FAILED'
            self.message = 'PX4 拒绝命令：' + self.last_ack
            self.pending.clear()
        elif m.result == 0:
            self.pending.discard(m.command)
            if m.command == VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
                self.origin_ready = True

    def tick(self):
        if not self.busy:
            return
        b = self.b
        age = time.monotonic() - self.started
        if age > (45 if self.state == 'LANDING' else 35):
            self.state = 'FAILED'
            self.message = '飞手操作超时；请检查当前飞控状态'
            self.pending.clear()
            return
        if not b.vehicle or not b.land or b.now() - b.state_times.get('vehicle', -1e9) > 2:
            return
        armed = b.vehicle.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.state == 'PREPARING' and age > 4 and self.origin_ready and self.ready():
            if self.operation == 'takeoff':
                self.takeoff()
            else:
                self.state = 'ARMING'
                self.message = '请求解锁'
                self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1)
        elif self.state == 'TAKING_OFF' and (not b.healthy() or b.alignment is None):
            self.state = 'FAILED'
            self.message = '起飞期间定位或坐标对齐失效；停止发送后续模式命令'
            self.pending.clear()
        elif self.state == 'TAKING_OFF' and b.local and not b.land.landed:
            if b.vehicle.nav_state not in (VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF,
                                           VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                self.state = 'FAILED'
                self.message = '起飞被飞手或飞控模式切换中断'
            elif self.ground_z - b.local.z > self.height - .35 and abs(b.local.vz) < .3:
                self.send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1, param2=4, param3=3)
                self.state = 'HOLDING'
                self.message = '起飞完成，切换悬停'
        elif ((self.state == 'ARMING' and armed) or
              (self.state == 'DISARMING' and not armed) or
              (self.state == 'LANDING' and b.land.landed) or
              (self.state == 'HOLDING' and b.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)):
            self.state = 'DONE'
            self.message = {'arm':'已解锁', 'disarm':'已上锁', 'land':'已着陆',
                            'hold':'已进入悬停', 'takeoff':'起飞完成，已进入悬停'}[self.operation]
            self.pending.clear()
