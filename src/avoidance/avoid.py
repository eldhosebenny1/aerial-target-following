from typing import NamedTuple

import numpy as np

from src.utils import config


class AvoidanceCommand(NamedTuple):
    label: str
    vx: float  # None means "let the policy decide"
    vy: float
    color: tuple


CLEAR = AvoidanceCommand("CLEAR", None, None, (0, 255, 0))


def thirds_scores(depth_map):
    """Mean depth of the left, centre and right thirds of the frame.

    Depth Anything predicts relative inverse depth, so a higher score means
    the surface is closer to the camera.
    """
    third = depth_map.shape[1] // 3
    left = float(np.mean(depth_map[:, :third]))
    center = float(np.mean(depth_map[:, third:2 * third]))
    right = float(np.mean(depth_map[:, 2 * third:]))
    return left, center, right


class ObstacleAvoidance:
    """Centre-frame obstacle check with a small majority filter.

    A single frame has to be backed up by at least half the buffer before the
    override fires, which keeps depth flicker from twitching the setpoint.
    """

    def __init__(self, threshold=config.DANGER_THRESHOLD, buffer_size=config.BUFFER_SIZE):
        self.threshold = threshold
        self.buffer_size = buffer_size
        self.history = []

    def evaluate(self, depth_map):
        left, center, right = thirds_scores(depth_map)

        self.history.append(center > self.threshold)
        if len(self.history) > self.buffer_size:
            self.history.pop(0)

        if sum(self.history) >= (self.buffer_size // 2 + 1):
            red = (0, 0, 255)
            if left > self.threshold and right > self.threshold:
                return AvoidanceCommand("REVERSE", -0.5, 0.0, red)
            if left < right:
                return AvoidanceCommand("VEER LEFT", 0.3, 0.5, red)
            return AvoidanceCommand("VEER RIGHT", 0.3, -0.5, red)

        if center > self.threshold * 0.7:
            return AvoidanceCommand("SLOW", 0.2, 0.0, (0, 165, 255))

        return CLEAR
