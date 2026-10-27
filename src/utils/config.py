"""All tunables in one place. Values match the last flown configuration."""

import os

# Weights are not committed, so both paths can be overridden from the shell.
MODEL_PATH = os.environ.get("MODEL_PATH", "models/best.pt")
POLICY_PATH = os.environ.get("POLICY_PATH", "models/policy.pt")

# PX4 SITL forwards MAVLink here when started with the default udp endpoint.
CONNECTION_URL = "udpin://0.0.0.0:14550"

VIDEO_SOURCE = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
ROTATE_FRAME = True  # camera is mounted sideways on the airframe

CONF_THRESHOLD = 0.25
IMG_SIZE = 800

# Pinhole distance estimate: distance_m = REAL_WIDTH_CM * FOCAL_LENGTH_PX / box_width_px / 100
REAL_WIDTH_CM = 36.4
FOCAL_LENGTH_PX = 945.0
SMOOTHING_FACTOR = 0.2

SAFE_ALTITUDE = 3.0
MAX_SPEED = 0.7  # m/s, scales the policy output
MAX_ACCEL = 0.08  # m/s change allowed per control iteration
MAX_YAW = 15.0  # deg/s, scales the policy output
TAKEOFF_SETTLE_TIME = 10  # seconds between takeoff and offboard start

DANGER_THRESHOLD = 0.3551537100918341
BUFFER_SIZE = 5

DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Small-hf"
DEPTH_INPUT_SIZE = 320
