"""从真实导航遥测生成航迹、目标编号和机体状态标签。"""
from collections import deque
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from uav_interfaces.msg import NavigationStatus
from visualization_msgs.msg import Marker, MarkerArray


class NavigationVisuals(Node):
    def __init__(self, **node_options):
        super().__init__('navigation_visuals', **node_options)
        self.vehicle_label = self.declare_parameter('vehicle_label', 'X500').value
        self.body_shape = self.declare_parameter('body_shape', 'cylinder').value
        if self.body_shape not in ('box', 'cylinder'):
            raise ValueError('body_shape must be box or cylinder')
        self.envelope = tuple(self.declare_parameter(k, -1.).value for k in
                              ('body_radius', 'body_above', 'body_below'))
        self.history = deque(maxlen=2500)
        self.odom = None
        self.phase = 'WAITING'
        self.session = None
        self.diag = {}
        self.goals = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/navigation/state_markers', latched)
        self.path = self.create_publisher(Path, '/navigation/flight_path', latched)
        self.create_subscription(Odometry, '/navigation/odometry', self.receive, 10)
        self.create_subscription(NavigationStatus, '/navigation/status', self.status, latched)
        self.create_subscription(String, '/navigation/control_diagnostics', self.diagnostics, 10)
        self.create_subscription(Path, '/navigation/staged_goals', lambda m:setattr(self,'goals',m.poses), latched)
        self.create_timer(.2, self.publish)

    def status(self, msg):
        if self.session is not None and self.session != msg.session:
            self.history.clear()
        self.session = msg.session
        self.phase = msg.phase

    def diagnostics(self, msg):
        try:self.diag = json.loads(msg.data)
        except ValueError:pass

    def receive(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.history:
            previous = self.history[-1]
            old_t = previous.header.stamp.sec + previous.header.stamp.nanosec * 1e-9
            if t < old_t:self.history.clear()
            elif t-old_t < .1:return
        sample = PoseStamped();sample.header=msg.header;sample.pose=msg.pose.pose
        self.history.append(sample)

    def publish(self):
        if not self.odom:return
        out=MarkerArray()
        clear=Marker();clear.action=Marker.DELETEALL;out.markers.append(clear)
        def marker(identifier,kind,position,scale,color,text=''):
            m=Marker();m.header.frame_id='odom';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='navigation';m.id=identifier;m.type=kind;m.action=Marker.ADD
            m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=position
            m.scale.x=m.scale.y=m.scale.z=scale
            m.color.r,m.color.g,m.color.b,m.color.a=color;m.text=text;out.markers.append(m)
        for i,pose in enumerate(self.goals[:100]):
            p=pose.pose.position
            marker(i*2,Marker.SPHERE,(p.x,p.y,p.z),.22,(.85,.4,1.,1.))
            marker(i*2+1,Marker.TEXT_VIEW_FACING,(p.x,p.y,p.z+.4),.22,(1.,.85,1.,1.),f'Goal {i+1}: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})')
        p=self.odom.pose.pose.position;v=self.odom.twist.twist.linear
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        mode={2:'POSCTL',4:'HOLD',14:'OFFBOARD',17:'TAKEOFF',18:'LAND'}.get(self.diag.get('px4_mode'),'PX4')
        marker(1000,Marker.TEXT_VIEW_FACING,(p.x,p.y,p.z+.6),.06,(.8,1.,1.,1.),f'{self.vehicle_label} | {mode} | {self.phase}\nz={p.z:.2f} m  v={speed:.2f} m/s')
        radius,above,below=self.envelope
        if min(self.envelope)>0:
            # World-upright envelope already includes the configured attitude allowance.
            marker(2000,Marker.LINE_LIST,(p.x,p.y,p.z),.012,(1.,1.,1.,.8))
            wire=out.markers[-1]
            if self.body_shape == 'box':
                corners = ((-radius, -radius), (radius, -radius),
                           (radius, radius), (-radius, radius))
                for height in (-below, above):
                    for i in range(4):
                        for j in (i, (i + 1) % 4):
                            x, y = corners[j]
                            wire.points.append(Point(x=x, y=y, z=height))
                for x, y in corners:
                    for height in (-below, above):
                        wire.points.append(Point(x=x, y=y, z=height))
            else:
                for height in (-below,above):
                    for i in range(48):
                        for j in (i,i+1):
                            angle=2*math.pi*j/48
                            wire.points.append(Point(x=radius*math.cos(angle),y=radius*math.sin(angle),z=height))
                for i in range(4):
                    angle=math.pi*i/2
                    for height in (-below,above):
                        wire.points.append(Point(x=radius*math.cos(angle),y=radius*math.sin(angle),z=height))
            marker(2001,Marker.SPHERE,(p.x,p.y,p.z),.07,(1.,1.,1.,1.))
        self.markers.publish(out)
        path=Path();path.header=self.odom.header;path.poses=list(self.history);self.path.publish(path)


def main():
    rclpy.init();node=NavigationVisuals()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
