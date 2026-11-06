"""Read-only ground preflight: no services, commands or PX4 input publishers."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
import yaml

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from uav_interfaces.msg import NavigationHealth
from px4_msgs.msg import (
    VehicleStatus, VehicleLocalPosition, VehicleOdometry,
    EstimatorStatusFlags, FailsafeFlags, TimesyncStatus,
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--scope', choices=['links', 'preflight'], default='preflight',
                        help='links reports data/fusion/alignment only; RC gates remain enforced')
    args = parser.parse_args()
    if not 5. <= args.duration <= 120.:
        parser.error('duration must be 5..120 seconds')
    source_config = Path(__file__).resolve().parents[2] / 'config/real/odin1.yaml'
    cloud_topic = yaml.safe_load(source_config.read_text())['uav_state_adapter']['ros__parameters']['cloud_topic']
    rclpy.init()
    node = rclpy.create_node('odin_readonly_preflight')
    records = {}
    specs = [
        ('/odin1/odometry_highfreq', Odometry, 20., .25),
        (cloud_topic, PointCloud2, 5., .5),
        ('/navigation/odometry', Odometry, 20., .25),
        ('/navigation/cloud', PointCloud2, 5., .5),
        ('/navigation/health', NavigationHealth, 5., .3),
        ('/navigation/control_diagnostics', String, 10., .3),
        ('/fmu/out/vehicle_status', VehicleStatus, 1., 2.),
        ('/fmu/out/vehicle_local_position', VehicleLocalPosition, 10., .3),
        ('/fmu/out/estimator_status_flags', EstimatorStatusFlags, .5, 2.),
        ('/fmu/out/failsafe_flags', FailsafeFlags, .5, 2.),
        ('/fmu/out/timesync_status', TimesyncStatus, .5, 2.),
        ('/fmu/in/vehicle_visual_odometry', VehicleOdometry, 20., .25),
        ('/fmu/in/offboard_control_mode', OffboardControlMode, 0., None),
        ('/fmu/in/trajectory_setpoint', TrajectorySetpoint, 0., None),
        ('/fmu/in/vehicle_command', VehicleCommand, 0., None),
    ]

    def receive(topic, msg):
        now = time.monotonic()
        rec = records.setdefault(topic, dict(count=0, first=now, max_gap=0.,
                                            max_header_age=None, backwards=0))
        if rec['count']:
            rec['max_gap'] = max(rec['max_gap'], now - rec['last'])
        rec.update(count=rec['count'] + 1, last=now, msg=msg)
        stamp = None
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            rec['max_header_age'] = max(rec['max_header_age'] or age, age)
            rec['min_header_age'] = min(rec.get('min_header_age', age), age)
        elif hasattr(msg, 'timestamp_sample'):
            stamp = msg.timestamp_sample / 1e6
        elif hasattr(msg, 'timestamp'):
            stamp = msg.timestamp / 1e6
        if stamp is not None:
            rec['backwards'] += int(stamp < rec.get('stamp', stamp))
            rec['stamp'] = stamp

    subscriptions = [node.create_subscription(
        kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data
    ) for topic, kind, _, _ in specs]
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    now = time.monotonic()
    result = dict(checked_at=datetime.now().astimezone().isoformat(),
                  duration_s=args.duration, topics={}, checks={})
    checks = result['checks']
    for topic, _, minimum, limit in specs:
        rec = records.get(topic)
        publishers = node.count_publishers(topic)
        value = dict(publishers=publishers, count=rec['count'] if rec else 0)
        if rec:
            value.update(hz=round((rec['count']-1) / max(rec['last']-rec['first'], .001), 2),
                         age_s=round(now-rec['last'], 3), max_gap_s=round(rec['max_gap'], 3),
                         timestamp_backwards=rec['backwards'])
            if rec['max_header_age'] is not None:
                value['header_age_range_s'] = [rec['min_header_age'], rec['max_header_age']]
        result['topics'][topic] = value
        if limit is not None:
            checks[topic] = bool(rec and publishers == 1 and value['hz'] >= minimum
                                 and value['age_s'] < limit and rec['max_gap'] < limit
                                 and not rec['backwards'] and (rec['max_header_age'] is None
                                 or (-.05 <= rec['min_header_age'] and rec['max_header_age'] < limit)))

    def last(topic):
        return records.get(topic, {}).get('msg')

    vehicle = last('/fmu/out/vehicle_status')
    checks['disarmed'] = bool(vehicle and vehicle.arming_state == vehicle.ARMING_STATE_DISARMED)
    checks['px4_no_failsafe'] = bool(vehicle and not vehicle.failsafe)
    checks['px4_preflight_checks_pass'] = bool(vehicle and vehicle.pre_flight_checks_pass)
    local = last('/fmu/out/vehicle_local_position')
    checks['px4_position_velocity_valid'] = bool(local and all(
        getattr(local, k) for k in ['xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid']))
    ekf = last('/fmu/out/estimator_status_flags')
    checks['external_position_height_yaw_fused'] = bool(ekf and all(
        getattr(ekf, k) for k in ['cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw']))
    failsafe = last('/fmu/out/failsafe_flags')
    checks['rc_available'] = bool(failsafe and not failsafe.manual_control_signal_lost)
    health = last('/navigation/health')
    checks['odin_navigation_health'] = bool(health and health.source == 'odin1'
                                           and health.odometry_valid and health.cloud_valid)
    diag_msg = last('/navigation/control_diagnostics')
    diag = json.loads(diag_msg.data) if diag_msg else {}
    checks['ground_alignment'] = diag.get('aligned') is True
    checks['control_bridge_healthy'] = diag.get('healthy') is True
    checks['position_bridge_healthy'] = diag.get('position_healthy') is True
    result['control_diagnostics'] = diag
    result['navigation_reason'] = health.reason if health else 'no navigation health'
    # On the ground the control gate must not stream setpoints or issue commands.
    checks['no_ground_control_output'] = all(
        result['topics'][t]['count'] == 0 for t in
        ['/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_command'])
    result['ground_preflight_pass'] = all(checks.values())
    deferred = {'rc_available', 'px4_preflight_checks_pass', 'control_bridge_healthy'}
    result['link_checks_pass'] = all(passed for name, passed in checks.items() if name not in deferred)
    result['scope'] = args.scope
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    for name, passed in checks.items():
        label = 'DEFER ' if args.scope == 'links' and name in deferred else ('PASS ' if passed else 'FAIL ')
        print(label + name)
    print('reason: ' + (diag.get('blocking_reason') or result['navigation_reason']))
    print('GROUND PREFLIGHT: ' + ('PASS' if result['ground_preflight_pass'] else 'INCOMPLETE'))
    print('DATA/FUSION LINKS: ' + ('PASS' if result['link_checks_pass'] else 'INCOMPLETE'))
    node.destroy_node()
    rclpy.shutdown()
    passed = result['link_checks_pass'] if args.scope == 'links' else result['ground_preflight_pass']
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
