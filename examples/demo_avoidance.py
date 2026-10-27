"""Run the obstacle-avoidance state machine on a synthetic depth sequence.

No drone, no camera and no model weights are needed. Useful for seeing the
hysteresis behaviour before you change DANGER_THRESHOLD or BUFFER_SIZE.

    python examples/demo_avoidance.py
"""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.avoidance.avoid import ObstacleAvoidance
from src.control.limiter import VelocityLimiter

WIDTH, HEIGHT = 160, 120


def scene(obstacle=None):
    """Depth-Anything style map: higher value means nearer to the camera."""
    frame = np.tile(np.linspace(0.05, 0.30, WIDTH, dtype=np.float32), (HEIGHT, 1))
    if obstacle == "right":
        frame[:, 120:] = 0.9
    elif obstacle == "center":
        frame[:, 80:] = 0.9
    elif obstacle == "both":
        frame[:, :40] = 0.9
        frame[:, 120:] = 0.9
    return frame


sequence = (
    [("clear", None)] * 5
    + [("right edge", "right")] * 4      # outside the centre third, so no reaction
    + [("closing in", "center")] * 6     # SLOW, then the majority filter trips
    + [("clear", None)] * 5              # override releases once history drains
)

avoidance = ObstacleAvoidance()
limiter = VelocityLimiter()

for index, (name, obstacle) in enumerate(sequence):
    command = avoidance.evaluate(scene(obstacle))
    vx, vy = limiter.step(
        command.vx if command.vx is not None else 0.0,
        command.vy if command.vy is not None else 0.0,
    )
    print(f"{index:02d}  {name:<11} {command.label:<10} setpoint=({vx:+.2f}, {vy:+.2f})")

out = Path("outputs")
out.mkdir(exist_ok=True)
color = cv2.applyColorMap((scene("center") * 255).astype(np.uint8), cv2.COLORMAP_MAGMA)
cv2.putText(color, "centre obstacle: VEER LEFT", (6, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
cv2.imwrite(str(out / "avoidance_demo.png"), color)
print(f"wrote {out / 'avoidance_demo.png'}")
