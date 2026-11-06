"""连续航向生成：保留角速度限制，可选角加速度限制。"""
import math
from uav_state_adapter.core import wrap


class YawProfile:
    def __init__(self, acceleration=0., rate=1.):
        if not math.isfinite(acceleration) or acceleration < 0 or not math.isfinite(rate) or rate <= 0:
            raise ValueError("Invalid yaw profile limits")
        self.acceleration = acceleration
        self.rate_limit = rate
        self.yaw = 0.
        self.rate = 0.

    def reset(self, yaw):
        self.yaw = yaw
        self.rate = 0.

    def step(self, target, dt):
        if not math.isfinite(target) or not math.isfinite(dt) or dt <= 0:
            raise ValueError("Invalid yaw target or time step")
        error = wrap(target - self.yaw)
        if self.acceleration == 0:
            step = max(-self.rate_limit * dt, min(self.rate_limit * dt, error))
            self.rate = step / dt
        else:
            # 用剩余角度限制可停止的角速度，再按角加速度连续变化。
            desired = math.copysign(min(self.rate_limit,
                                       math.sqrt(2 * self.acceleration * abs(error))), error)
            delta = self.acceleration * dt
            self.rate += max(-delta, min(delta, desired - self.rate))
            step = self.rate * dt
        self.yaw = wrap(self.yaw + step)
        return self.yaw, self.rate
