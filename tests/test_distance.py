import pytest

from src.perception.distance import (
    DistanceSmoother,
    distance_from_width,
    lateral_offset_m,
)


def test_distance_matches_pinhole_formula():
    # 100 px box, 945 px focal length, 36.4 cm real width -> 3.44 m
    assert distance_from_width(100) == pytest.approx(3.4398, rel=1e-3)


def test_distance_falls_off_with_box_width():
    near = distance_from_width(300)
    far = distance_from_width(60)
    assert near == pytest.approx(far / 5, rel=1e-6)


def test_zero_width_box_has_no_estimate():
    assert distance_from_width(0) is None


def test_lateral_offset_in_metres():
    # 95 px off centre at 2 m and 945 px focal length
    assert lateral_offset_m(735, 640, 2.0) == pytest.approx(0.201, abs=1e-3)
    assert lateral_offset_m(640, 640, 2.0) == pytest.approx(0.0)


def test_smoother_moves_towards_measurement():
    smoother = DistanceSmoother(alpha=0.5, initial=4.0)
    assert smoother.update(6.0) == pytest.approx(5.0)
    assert smoother.update(6.0) == pytest.approx(5.5)


def test_smoother_starts_at_three_metres():
    assert DistanceSmoother().value == pytest.approx(3.0)
