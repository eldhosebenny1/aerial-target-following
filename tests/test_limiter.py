import pytest

from src.control.limiter import VelocityLimiter


def test_ramps_towards_target_at_max_rate():
    limiter = VelocityLimiter(max_accel=0.08)
    assert limiter.step(0.7, 0.0)[0] == pytest.approx(0.08)
    assert limiter.step(0.7, 0.0)[0] == pytest.approx(0.16)


def test_stops_at_target_instead_of_overshooting():
    limiter = VelocityLimiter(max_accel=0.5)
    assert limiter.step(0.3, 0.0)[0] == pytest.approx(0.3)


def test_reversal_is_slow_from_full_speed():
    limiter = VelocityLimiter(max_accel=0.08)
    for _ in range(20):
        limiter.step(1.0, 0.0)
    assert limiter.vx == pytest.approx(1.0)
    # One iteration only gives back 0.08 m/s, so a full reversal takes seconds.
    assert limiter.step(-1.0, 0.0)[0] == pytest.approx(0.92)


def test_axes_are_independent():
    limiter = VelocityLimiter(max_accel=0.08)
    vx, vy = limiter.step(0.7, -0.7)
    assert (vx, vy) == (pytest.approx(0.08), pytest.approx(-0.08))


def test_reset_returns_to_hover():
    limiter = VelocityLimiter(max_accel=0.5)
    limiter.step(0.5, 0.5)
    limiter.reset()
    assert (limiter.vx, limiter.vy) == (0.0, 0.0)
