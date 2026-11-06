"""Supervise a read-only rosbag recorder; preserve parameters and review evidence."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import ListParameters, GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from uav_interfaces.msg import NavigationStatus, NavigationHealth
from std_msgs.msg import String
import yaml


def wall_time():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()


def boot_identity():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return 'unavailable'


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        stream.write(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def storage_files(directory):
    files = []
    for path in sorted(directory.glob('*')):
        try:
            if path.is_file():
                files.append({'name': path.name, 'bytes': path.stat().st_size})
        except OSError:
            pass  # WAL/journal files can disappear between enumeration and stat
    return files


def bag_summary(bag):
    metadata = bag / 'metadata.yaml'
    if not metadata.exists():
        return {'finalized': False, 'topic_counts': {}}
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: t['message_count']
              for t in info['topics_with_message_count']}
    return {'finalized': True, 'topic_counts': counts,
            'message_count': info['message_count'],
            'duration_s': info['duration']['nanoseconds'] / 1e9,
            'files': info['relative_file_paths']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('recorder command is required after --')
    args.metadata.mkdir(parents=True, exist_ok=True)
    args.bag.parent.mkdir(parents=True, exist_ok=True)
    minimum = float(os.environ.get('SCUT_RECORD_MIN_FREE_GB', '2')) * 1024**3
    if not (0 <= minimum < float('inf')):
        parser.error('SCUT_RECORD_MIN_FREE_GB must be finite and nonnegative')
    if shutil.disk_usage(args.bag.parent).free < minimum:
        parser.error('insufficient free disk space for recording')
    stopped = []
    def request_stop(signum, _frame):
        if not stopped:
            stopped.append(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    start = wall_time()
    write_json(args.metadata / 'session.json', {
        'started_at': start, 'bag': str(args.bag), 'command': command,
        'boot_id': boot_identity(),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'minimum_free_bytes': minimum})
    print(f'录包目录：{args.bag}\n复盘资料：{args.metadata}\n飞行结束后按 Ctrl+C 正常关闭。', flush=True)
    output = (args.metadata / 'recorder.log').open('w')
    events = (args.metadata / 'events.jsonl').open('w', buffering=1)
    child = None
    node = None
    subscriptions = []
    parameter_clients = []
    pending = []
    previous = {}
    snapshots = {}
    errors = []
    fault_paths = set()
    stop_sent = None
    interrupted = False
    exit_code = 1

    def event(kind, value):
        events.write(json.dumps({'received_at': wall_time(), 'kind': kind,
                                 'value': value}, ensure_ascii=False) + '\n')
        if kind == '/navigation/localization_fault' or (
                kind == '/navigation/status' and isinstance(value, dict)
                and value.get('phase') in ('FAULT', 'CANCELLED', 'COMPLETE')):
            events.flush()
            os.fsync(events.fileno())

    def parameters(label):
        graph = [{'topic': t, 'types': types, 'publishers': node.count_publishers(t)}
                 for t, types in node.get_topic_names_and_types()]
        snapshots[label] = {'at': wall_time(), 'topics': graph, 'parameters': {}}
        for name in ('uav_state_adapter', 'ego_planner_node', 'traj_server', 'uav_px4_bridge'):
            client = node.create_client(ListParameters, f'/{name}/list_parameters')
            parameter_clients.append(client)
            if client.service_is_ready():
                req = ListParameters.Request(); req.depth = 0
                pending.append((client.call_async(req), label, name, 'list', None))
            else:
                snapshots[label]['parameters'][name] = {'unavailable': True}

    def drain_parameters():
        for job in pending[:]:
            future, label, name, phase, names = job
            if not future.done():
                continue
            pending.remove(job)
            try:
                result = future.result()
                if phase == 'list':
                    names = result.result.names
                    client = node.create_client(GetParameters, f'/{name}/get_parameters')
                    parameter_clients.append(client)
                    req = GetParameters.Request(); req.names = names
                    pending.append((client.call_async(req), label, name, 'get', names))
                else:
                    snapshots[label]['parameters'][name] = {
                        k: message_to_ordereddict(v) for k, v in zip(names, result.values)}
            except Exception as exc:
                snapshots[label]['parameters'][name] = {'error': str(exc)}
        write_json(args.metadata / 'runtime_snapshots.json', snapshots)

    try:
        # Separate group: terminal Ctrl+C is forwarded exactly once by us.
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('flight_record_observer')
        def receive(msg, topic):
            value = message_to_ordereddict(msg)
            if isinstance(msg, String):
                try:
                    value = json.loads(msg.data)
                except ValueError:
                    value = msg.data
            if (topic == '/navigation/localization_fault' and isinstance(value, dict)
                    and isinstance(value.get('snapshot_path'), str)):
                fault_paths.add(value['snapshot_path'])
            if topic == '/navigation/control_diagnostics' and isinstance(value, dict):
                key = {k: value.get(k) for k in ('phase', 'reason', 'control_fault',
                       'executor_timeout', 'localization_fault', 'blocking_reason',
                       'armed', 'px4_mode', 'px4_connected')}
            elif topic == '/navigation/health':
                key = {k: value.get(k) for k in ('epoch', 'odometry_valid', 'cloud_valid', 'reason')}
            else:
                key = {k: v for k, v in value.items() if k != 'header'} if isinstance(value, dict) else value
            if previous.get(topic) != key:
                event(topic, value); previous[topic] = key
        for topic, kind, retained in [('/navigation/status', NavigationStatus, True),
                ('/navigation/health', NavigationHealth, False),
                ('/navigation/localization_fault', String, True),
                ('/navigation/control_diagnostics', String, False),
                ('/planning/runtime_diagnostics', String, False),
                ('/planning/diagnostics', String, True)]:
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if retained else DurabilityPolicy.VOLATILE)
            subscriptions.append(node.create_subscription(kind, topic,
                lambda m, t=topic: receive(m, t), qos))
        began = time.monotonic()
        start_snapshot = False
        next_disk_check = began
        while child.poll() is None:
            now = time.monotonic()
            if not start_snapshot and now - began >= 2:
                parameters('start'); start_snapshot = True
            if now >= next_disk_check:
                next_disk_check = now + 2
                free_bytes = shutil.disk_usage(args.bag.parent).free
                write_json(args.metadata / 'recorder_health.json', {
                    'at': wall_time(), 'monotonic_ns': time.monotonic_ns(),
                    'boot_id': boot_identity(), 'recorder_pid': child.pid,
                    'free_bytes': free_bytes,
                    'storage_files': storage_files(args.bag)})
                events.flush()
                os.fsync(events.fileno())
                if free_bytes < minimum and not stopped:
                    stopped.append('disk_space_low')
            if stopped and stop_sent is None:
                parameters('stop')
                event('stop_requested', stopped[0])
                os.killpg(child.pid, signal.SIGINT)
                stop_sent = now
            if stop_sent is not None and now - stop_sent > 30 and not interrupted:
                os.killpg(child.pid, signal.SIGTERM)
                interrupted = True
                errors.append('recorder did not finalize within 30 s; SIGTERM sent')
            if stop_sent is not None and now - stop_sent > 40:
                os.killpg(child.pid, signal.SIGKILL)
                errors.append('recorder force killed; bag may require recovery')
                child.wait()
                break
            rclpy.spin_once(node, timeout_sec=.05)
            if pending:
                drain_parameters()
        exit_code = child.wait()
        if 'stop' not in snapshots:
            parameters('stop')
        deadline = time.monotonic() + 2
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05); drain_parameters()
    except Exception as exc:
        errors.append(repr(exc))
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
                errors.append('cleanup force killed recorder')
        for _, label, name, _, _ in pending:
            snapshots[label]['parameters'].setdefault(name, {'incomplete': True})
        write_json(args.metadata / 'runtime_snapshots.json', snapshots)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.close(); events.close()
    archived_faults = []
    default_fault_root = (Path(os.environ.get('SCUT_ROOT', os.getcwd())) / 'run/odin_faults').resolve()
    for filename in sorted(fault_paths):
        item = {'source': filename}
        try:
            source = Path(filename).resolve()
            if not source.is_relative_to(default_fault_root) or source.name != 'fault.json':
                item['status'] = 'custom_directory_not_auto_archived'
            elif not source.is_file():
                item['status'] = 'source_not_yet_written'
            elif source.stat().st_size > 32 * 1024**2:
                item['status'] = 'snapshot_exceeds_size_limit'
            else:
                destination = args.metadata / 'fault_snapshots' / source.parent.name / 'fault.json'
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
                snapshot = json.loads(destination.read_text())
                item.update(status='copied', file=str(destination),
                            capture_finished=snapshot.get('capture_finished', False))
        except (OSError, ValueError) as exc:
            item.update(status='copy_failed', error=str(exc))
        archived_faults.append(item)
    summary = bag_summary(args.bag)
    required = ['/navigation/odometry', '/navigation/cloud', '/navigation/health',
                '/navigation/status', '/navigation/control_diagnostics', '/navigation/flight_path',
                '/planning/trajectory', '/planning/position_cmd', '/planning/safety',
                '/planning/execution_status', '/planning/runtime_diagnostics',
                '/grid_map/occupancy', '/grid_map/occupancy_inflate']
    missing = [t for t in required if not summary['topic_counts'].get(t)]
    summary.update(started_at=start, ended_at=wall_time(), recorder_exit_code=exit_code,
                   stop_reason=stopped[0] if stopped else 'recorder_exit',
                   missing_review_topics=missing, errors=errors,
                   localization_fault_snapshots=archived_faults)
    write_json(args.metadata / 'summary.json', summary)
    lines = ['# 飞行录包摘要', '', f'录包：{args.bag}', f'开始：{start}',
             f'结束：{summary["ended_at"]}', f'正常生成 metadata.yaml：{summary["finalized"]}',
             f'录包进程退出码：{exit_code}', '', '## 实际写入消息数', '', '| 话题 | 数量 |', '| --- | --- |']
    lines += [f'| {t} | {c} |' for t,c in sorted(summary['topic_counts'].items())]
    lines += ['', '## 缺少的复盘话题', '', *missing,
              '', '缺少话题不等同于通信故障：未执行任务时可能没有轨迹或设定点。',
              '事件接收时间见 events.jsonl；实际运行参数见 runtime_snapshots.json；录包日志见 recorder.log。',
              '该摘要仅检查记录完整性，不是飞行通过判定。', *errors]
    if archived_faults:
        lines += ['', '## 定位首故障快照', '', *[json.dumps(f, ensure_ascii=False) for f in archived_faults]]
    (args.metadata / 'review.md').write_text('\n'.join(lines) + '\n')
    print(f'录包结束；复盘摘要：{args.metadata / "review.md"}', flush=True)
    if missing:
        print('未录到的复盘话题：' + ', '.join(missing), flush=True)
    return 0 if summary['finalized'] and not errors and exit_code in (0, 130, -signal.SIGINT) else 1


if __name__ == '__main__':
    raise SystemExit(main())
