from queue import Empty, Queue
from threading import Thread

import cv2


class GUIThread:
    """OpenCV display on its own thread so imshow never stalls the event loop.

    Frames are dropped rather than queued when the display falls behind.
    """

    def __init__(self, max_queue_size=2):
        self.frame_queue = Queue(maxsize=max_queue_size)
        self.exit_flag = False
        self.thread = Thread(target=self._display_loop, daemon=True)
        self.thread.start()

    def _display_loop(self):
        cv2.namedWindow("Target Follower", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Map", cv2.WINDOW_NORMAL)

        while not self.exit_flag:
            try:
                main_frame, depth_frame = self.frame_queue.get(timeout=0.05)
            except Empty:
                continue

            if main_frame is not None:
                cv2.imshow("Target Follower", main_frame)

            if depth_frame is not None:
                cv2.imshow("Depth Map", depth_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.exit_flag = True

        cv2.destroyAllWindows()

    def update(self, main_frame, depth_frame):
        if self.exit_flag:
            return
        try:
            self.frame_queue.put_nowait((main_frame, depth_frame))
        except Exception:
            pass

    def stop(self):
        self.exit_flag = True
        self.thread.join(timeout=1)
