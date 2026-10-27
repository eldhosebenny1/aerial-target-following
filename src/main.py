"""Entry point: camera -> detection/tracking -> depth -> policy -> PX4 offboard."""

import asyncio
import time

import cv2
import numpy as np
import torch
from mavsdk.offboard import OffboardError

from src.avoidance.avoid import ObstacleAvoidance
from src.control.flight import FlightController
from src.control.limiter import VelocityLimiter
from src.navigation.policy import TargetPolicy
from src.perception.camera import Camera
from src.perception.depth import DepthEstimator
from src.perception.detector import TargetTracker
from src.perception.distance import (
    DistanceSmoother,
    distance_from_width,
    lateral_offset_m,
)
from src.utils import config
from src.utils.gui import GUIThread


def target_error(frame, target, smoother):
    """Target offset as [right, up, forward] metres, relative to the camera axis."""
    x1, y1, x2, y2 = target.box

    distance = distance_from_width(x2 - x1)
    if distance is None:
        return None

    smoothed = smoother.update(distance)
    lateral = lateral_offset_m((x1 + x2) / 2.0, frame.shape[1] / 2.0, smoothed)
    return [lateral, 0.0, smoothed]


def draw_target(canvas, target, distance):
    x1, y1, x2, y2 = [int(value) for value in target.box]

    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(
        canvas,
        f"ID:{target.track_id} {distance:.2f}m",
        (x1, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )


def draw_status(canvas, command, distance, fps):
    cv2.putText(canvas, f"FPS: {fps:.1f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, f"Dist: {distance:.2f}m", (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, f"Obstacle: {command.label}", (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, command.color, 2)


async def run():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Running on: {device.upper()}")

    tracker = TargetTracker(config.MODEL_PATH, device, config.CONF_THRESHOLD, config.IMG_SIZE)
    policy = TargetPolicy(config.POLICY_PATH, device)
    depth = DepthEstimator(device)

    avoidance = ObstacleAvoidance()
    limiter = VelocityLimiter()
    smoother = DistanceSmoother()
    gui = GUIThread()

    flight = FlightController()
    await flight.connect()
    await flight.takeoff()

    try:
        await flight.start_offboard()
    except OffboardError as error:
        print(f"Offboard start failed: {error}")
        await flight.land()
        gui.stop()
        return

    camera = Camera()
    previous_time = time.time()

    try:
        while not gui.exit_flag:
            frame = camera.read()
            if frame is None:
                print("Camera frame not received")
                break

            canvas = frame.copy()
            target = tracker.update(frame)

            error = None
            if target is not None:
                error = target_error(frame, target, smoother)
                if error is not None:
                    draw_target(canvas, target, error[2])

            depth_map = depth.infer(frame)
            command = avoidance.evaluate(depth_map)

            # Policy runs only when a target is visible and the path is clear.
            if error is not None and command.vx is None:
                ned_velocity = await flight.velocity_ned()
                target_vx, target_vy, target_yaw = policy.act(ned_velocity, error)
            else:
                target_vx = 0.0
                target_vy = 0.0
                target_yaw = 0.0

            if command.vx is not None:
                target_vx = command.vx
                target_vy = command.vy
                target_yaw = 0.0

            vx, vy = limiter.step(target_vx, target_vy)
            await flight.set_velocity(vx, vy, target_yaw)

            current_time = time.time()
            fps = 1.0 / (current_time - previous_time + 1e-6)
            previous_time = current_time

            draw_status(canvas, command, smoother.value, fps)

            depth_color = cv2.applyColorMap(
                (depth_map * 255).astype(np.uint8), cv2.COLORMAP_MAGMA
            )
            cv2.putText(depth_color, f"NAV: {command.label}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, command.color, 2)

            gui.update(canvas, depth_color)
            await asyncio.sleep(0.001)

    finally:
        print("Landing and cleaning up...")
        gui.stop()
        await flight.stop_offboard()
        await flight.land()
        camera.release()


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("User interrupted.")
