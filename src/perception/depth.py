import cv2
import numpy as np
from PIL import Image
from transformers import pipeline

from src.utils import config


class DepthEstimator:
    """Monocular relative depth from Depth Anything V2 (small checkpoint)."""

    def __init__(self, device, model=config.DEPTH_MODEL, size=config.DEPTH_INPUT_SIZE):
        self.pipe = pipeline("depth-estimation", model=model, device=device)
        self.size = size

    def infer(self, frame_bgr):
        small = cv2.resize(frame_bgr, (self.size, self.size))
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        result = self.pipe(Image.fromarray(rgb))
        return normalise(np.array(result["depth"], dtype=np.float32))


def normalise(depth_map):
    """Min-max to 0..1. Per-frame, so the scale is not metric or stable."""
    low = depth_map.min()
    high = depth_map.max()
    return (depth_map - low) / (high - low + 1e-6)
