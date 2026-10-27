import numpy as np
import pytest

from src.avoidance.avoid import CLEAR, ObstacleAvoidance, thirds_scores


def flat(value, shape=(30, 30)):
    return np.full(shape, value, dtype=np.float32)


def test_scores_are_per_third():
    frame = flat(0.1)
    frame[:, :10] = 0.9
    left, center, right = thirds_scores(frame)
    assert (left, center, right) == (
        pytest.approx(0.9),
        pytest.approx(0.1),
        pytest.approx(0.1),
    )


def test_clear_path_lets_the_policy_drive():
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    command = avoid.evaluate(flat(0.1))
    assert command is CLEAR
    assert command.vx is None and command.vy is None


def test_single_blocked_frame_is_not_enough():
    # Hard override needs 3 of 5 frames, so the first one only triggers SLOW.
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    assert avoid.evaluate(flat(0.9)).label == "SLOW"


def test_majority_of_blocked_frames_override():
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    for _ in range(3):
        command = avoid.evaluate(flat(0.9))
    assert command.label == "REVERSE"
    assert command.vx < 0


def test_override_clears_once_history_drains():
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    for _ in range(5):
        avoid.evaluate(flat(0.9))
    for _ in range(3):
        command = avoid.evaluate(flat(0.1))
    assert command is CLEAR


def test_near_band_slows_down_without_override():
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    command = avoid.evaluate(flat(0.4))  # above 0.7 * threshold
    assert command.label == "SLOW"
    assert command.vx == 0.2
    assert command.vy == 0.0


def test_obstacle_on_the_left_steers_right():
    frame = flat(0.1)
    frame[:, :20] = 0.9  # left and centre thirds are blocked
    avoid = ObstacleAvoidance(threshold=0.5, buffer_size=5)
    for _ in range(3):
        command = avoid.evaluate(frame)
    assert command.label == "VEER RIGHT"
    assert command.vy == -0.5
