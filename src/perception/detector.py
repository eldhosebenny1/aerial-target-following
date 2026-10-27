from dataclasses import dataclass

import numpy as np
from ultralytics import YOLO


@dataclass
class Target:
    box: np.ndarray  # x1, y1, x2, y2 in pixels
    track_id: int


class TargetTracker:
    """YOLO detections with ByteTrack identity persistence across frames."""

    def __init__(self, weights, device, conf=0.25, imgsz=800):
        self.model = YOLO(weights)
        self.model.to(device)
        self.conf = conf
        self.imgsz = imgsz

    def update(self, frame):
        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
        )
        return self._select(results[0])

    @staticmethod
    def _select(result):
        boxes = result.boxes
        if boxes is None or boxes.id is None or len(boxes) == 0:
            return None

        box = boxes.xyxy.cpu().numpy()[0]
        track_id = int(boxes.id.int().cpu().tolist()[0])
        return Target(box=box, track_id=track_id)
