"""Bounded raw-odometry black box; disk writes never run in the ROS callback."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import os
from pathlib import Path
import time


class FaultRecorder:
    def __init__(self, path, on_error=None, before_s=2., after_s=2., max_samples=2400):
        self.path = Path(path)
        self.before_s, self.after_s, self.max_samples = before_s, after_s, max_samples
        self.samples = deque(maxlen=max_samples)
        self.report = None
        self.deadline = None
        self.closed = False
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='odom-fault-save')
        self.on_error = on_error or (lambda error: None)

    def push(self, sample, wall=None):
        wall = time.monotonic() if wall is None else wall
        self.samples.append((wall, sample))
        while self.samples and wall-self.samples[0][0] > self.before_s:
            self.samples.popleft()
        if self.report is not None and not self.report['capture_finished']:
            if wall <= self.deadline and len(self.report['raw_after']) < self.max_samples:
                self.report['raw_after'].append(sample)
            self.poll(wall)

    def trigger(self, evidence, wall=None):
        if self.report is not None:
            return  # preserve the first fault, including if subsequent data go stale
        wall = time.monotonic() if wall is None else wall
        self.deadline = wall+self.after_s
        self.report = dict(schema_version=1, fault=evidence,
                           requested_before_s=self.before_s, requested_after_s=self.after_s,
                           capture_finished=False, raw_before=[s for t,s in self.samples
                               if wall-t <= self.before_s], raw_after=[])
        self._save()  # save prehistory immediately; do not wait for post-fault samples

    def poll(self, wall=None):
        wall = time.monotonic() if wall is None else wall
        if self.report is not None and not self.report['capture_finished'] and wall >= self.deadline:
            self.report['capture_finished'] = True
            self.report['finish_reason'] = 'post_window_elapsed'
            self._save()

    def _save(self):
        report = copy.deepcopy(self.report)
        future = self.pool.submit(self._write, report)
        def done(f):
            if f.exception() is not None:
                self.on_error(str(f.exception()))
        future.add_done_callback(done)

    def _write(self, report):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix('.tmp')
        with temp.open('w') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, self.path)
        fd = os.open(str(self.path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.report is not None and not self.report['capture_finished']:
            self.report['capture_finished'] = True
            self.report['finish_reason'] = 'node_shutdown_before_post_window'
            self._save()
        self.pool.shutdown(wait=True)
