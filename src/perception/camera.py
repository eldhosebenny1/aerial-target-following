import cv2

from src.utils import config


class Camera:
    def __init__(self, source=config.VIDEO_SOURCE, width=config.FRAME_WIDTH,
                 height=config.FRAME_HEIGHT, rotate=config.ROTATE_FRAME):
        self.capture = cv2.VideoCapture(source)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.rotate = rotate

    def read(self):
        ok, frame = self.capture.read()
        if not ok:
            return None
        if self.rotate:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return frame

    def release(self):
        self.capture.release()
