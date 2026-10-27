from src.utils import config


class VelocityLimiter:
    """Slew-rate limit on the horizontal setpoint.

    The step is a fixed delta per control iteration, not per second, so the
    effective acceleration depends on the loop rate.
    """

    def __init__(self, max_accel=config.MAX_ACCEL):
        self.max_accel = max_accel
        self.vx = 0.0
        self.vy = 0.0

    def step(self, target_vx, target_vy):
        self.vx += _clamp(target_vx - self.vx, self.max_accel)
        self.vy += _clamp(target_vy - self.vy, self.max_accel)
        return self.vx, self.vy

    def reset(self):
        self.vx = 0.0
        self.vy = 0.0


def _clamp(value, limit):
    return max(-limit, min(limit, value))
