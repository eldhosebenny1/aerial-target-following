from src.utils import config


def distance_from_width(box_width_px, real_width_cm=config.REAL_WIDTH_CM,
                        focal_px=config.FOCAL_LENGTH_PX):
    """Pinhole model. Returns None if the box has collapsed to zero width."""
    if box_width_px <= 0:
        return None
    return real_width_cm * focal_px / box_width_px / 100.0


def lateral_offset_m(target_center_x, frame_center_x, distance_m,
                     focal_px=config.FOCAL_LENGTH_PX):
    """Horizontal pixel offset of the box centre, converted to metres."""
    return (target_center_x - frame_center_x) / focal_px * distance_m


class DistanceSmoother:
    """Exponential moving average, seeded with the 3 m start value."""

    def __init__(self, alpha=config.SMOOTHING_FACTOR, initial=3.0):
        self.alpha = alpha
        self.value = initial

    def update(self, measurement):
        self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value
