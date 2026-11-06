"""ROS-independent geometry, timing and trajectory validation (SI, XYZW)."""

import math
import numpy as np


def vector(value, size=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError("invalid finite vector")
    return a


def rotation(q):
    x, y, z, w = vector(q, 4)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion(m):
    m = np.asarray(m, float)
    out = np.empty(4)
    tr = np.trace(m)
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        out[:3] = [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]
        out[:3] /= s
        out[3] = s / 4
    else:
        i = int(np.argmax(np.diag(m)))
        j = (i + 1) % 3
        k = (i + 2) % 3
        s = 2 * math.sqrt(max(0, 1 + m[i, i] - m[j, j] - m[k, k]))
        out[i] = s / 4
        out[j] = (m[j, i] + m[i, j]) / s
        out[k] = (m[k, i] + m[i, k]) / s
        out[3] = (m[k, j] - m[j, k]) / s
    return out / np.linalg.norm(out)


def body_state(
    p, q, v, omega, sensor_translation, sensor_quaternion, velocity_world=False
):
    """Input T_world_sensor and T_body_sensor; output body-origin world state."""
    rws = rotation(q)
    rbs = rotation(sensor_quaternion)
    rwb = rws @ rbs.T
    lever = rwb @ vector(sensor_translation)
    w = rws @ vector(omega)
    vw = vector(v) if velocity_world else rws @ vector(v)
    return vector(p) - lever, quaternion(rwb), vw - np.cross(w, lever), rwb.T @ w


def skew(v):
    x, y, z = vector(v)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def body_covariances(pose, twist, q, translation, mount, world, velocity_world):
    """转换已有 6x6 协方差；全零仍表示上游未提供，不虚构定位精度。

    twist 使用给定姿态的条件雅可比。上游没有提供完整状态交叉协方差，
    因此不能由此重建姿态不确定性对速度的全部影响。
    """
    rws = rotation(q)
    rwb = rws @ rotation(mount).T
    lever = rwb @ vector(translation)
    jp = np.eye(6)
    jp[:3, 3:] = skew(lever)
    rw = np.zeros((6, 6))
    rw[:3, :3] = rw[3:, 3:] = world
    jp = rw @ jp
    jv = np.zeros((6, 6))
    jv[:3, :3] = rwb.T @ (np.eye(3) if velocity_world else rws)
    jv[:3, 3:] = rwb.T @ skew(lever) @ rws
    jv[3:, 3:] = rwb.T @ rws
    result = []
    for values, jacobian in ((pose, jp), (twist, jv)):
        covariance = vector(values, 36).reshape(6, 6)
        result.append((jacobian @ covariance @ jacobian.T).reshape(-1).tolist())
    return result


ENU_NED = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
FLU_FRD = np.diag([1.0, -1, -1])


def px4_pose(p, q):
    return ENU_NED @ vector(p), quaternion(ENU_NED @ rotation(q) @ FLU_FRD)


def px4_variances(pose_covariance, twist_covariance, body_quaternion):
    """ROS world pose/body twist covariance -> PX4 world position/body attitude/world velocity.

    A missing (all-zero) covariance remains unknown (NaN). Reject malformed
    covariance rather than assigning the real device a fabricated precision.
    """
    def covariance(values):
        c = vector(values, 36).reshape(6, 6)
        if not np.any(c):
            return None
        if not np.allclose(c, c.T, atol=1e-8):
            raise ValueError("asymmetric covariance")
        c = (c + c.T) / 2
        if np.linalg.eigvalsh(c).min() < -1e-8:
            raise ValueError("negative covariance")
        return c

    pose, twist = covariance(pose_covariance), covariance(twist_covariance)
    r = rotation(body_quaternion)
    unknown = [float("nan")] * 3
    def diagonal(c, transform):
        return np.maximum(np.diag(transform @ c @ transform.T), 0.).tolist()
    return (
        diagonal(pose[:3, :3], ENU_NED) if pose is not None else unknown.copy(),
        diagonal(pose[3:, 3:], FLU_FRD @ r.T) if pose is not None else unknown.copy(),
        diagonal(twist[:3, :3], ENU_NED @ r) if twist is not None else unknown.copy(),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


def fresh(now, stamp, timeout, future=0.05):
    return (
        all(math.isfinite(x) for x in [now, stamp, timeout])
        and stamp > 0
        and -future <= now - stamp <= timeout
    )


class Polynomial:
    def __init__(self, durations, x, y, z, limits=None):
        self.durations = np.asarray(durations, float)
        if (
            self.durations.ndim != 1
            or not len(self.durations)
            or not np.isfinite(self.durations).all()
            or np.min(self.durations) <= 0
            or self.durations.sum() > 120
        ):
            raise ValueError("invalid trajectory durations")
        n = len(self.durations)
        arrays = [vector(c, 6 * n).reshape(n, 6) for c in [x, y, z]]
        self.c = np.stack(arrays, axis=1)
        self.duration = float(self.durations.sum())
        for i in range(n - 1):
            for d in range(3):
                if (
                    np.linalg.norm(
                        self.piece(i, self.durations[i], d) - self.piece(i + 1, 0, d)
                    )
                    > 0.02
                ):
                    raise ValueError("discontinuous polynomial")
        if limits:
            for i, T in enumerate(self.durations):
                for d, limit in enumerate(limits, 1):
                    # Exact extrema of squared derivative norm, including endpoints.
                    cs = [np.polyder(c, d) for c in self.c[i]]
                    sq = np.array([0.0])
                    for c in cs:
                        sq = np.polyadd(sq, np.polymul(c, c))
                    roots = np.roots(np.polyder(sq)) if np.any(sq) else []
                    ts = [0.0, T] + [
                        float(t.real)
                        for t in roots
                        if abs(t.imag) < 1e-7 and 0 < t.real < T
                    ]
                    if (
                        max(sum(float(np.polyval(c, t)) ** 2 for c in cs) for t in ts)
                        > (limit * 1.1) ** 2
                    ):
                        raise ValueError("dynamic limit exceeded")

    def piece(self, i, t, d=0):
        return np.array([np.polyval(np.polyder(c, d), t) for c in self.c[i]])

    def sample(self, t):
        t = min(max(float(t), 0.0), self.duration)
        i = 0
        while i < len(self.durations) - 1 and t > self.durations[i]:
            t -= self.durations[i]
            i += 1
        return tuple(self.piece(i, t, d) for d in range(4))


class ClockGuard:
    def __init__(self):
        self.last = None

    def update(self, t):
        reset = self.last is not None and (t < self.last - 1e-6 or t - self.last > 2.0)
        self.last = t
        return not reset
