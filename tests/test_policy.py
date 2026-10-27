from pathlib import Path

import pytest

torch = pytest.importorskip("torch")

from src.navigation.policy import TargetPolicy

POLICY_PATH = Path(__file__).parent.parent / "models" / "policy.pt"

pytestmark = pytest.mark.skipif(
    not POLICY_PATH.exists(), reason="policy.pt is not in the checkout"
)


def load_policy(**kwargs):
    return TargetPolicy(str(POLICY_PATH), device="cpu", **kwargs)


def test_observation_layout_matches_exported_policy():
    policy = load_policy()
    observation = policy.build_observation((0.1, -0.1, 0.0), (0.05, 0.0, 2.5))
    assert observation.shape == (15,)

    with torch.no_grad():
        # The exported actor is 15 -> 32 -> 32 -> 4.
        assert policy.policy(observation.unsqueeze(0)).shape == (1, 4)


def test_actions_respect_configured_limits():
    policy = load_policy(max_speed=0.7, max_yaw=15.0)
    vx, vy, yaw = policy.act((0.0, 0.0, 0.0), (0.0, 0.0, 20.0))
    assert abs(vx) <= 0.7
    assert abs(vy) <= 0.7
    assert abs(yaw) <= 15.0


def test_first_frame_rate_term_spikes():
    policy = load_policy()
    observation = policy.build_observation((0.0, 0.0, 0.0), (0.0, 0.0, 2.0))
    # last_error starts at zero, so the rate term jumps on the first frame.
    assert observation[14].item() == pytest.approx(2.0)


def test_rate_term_is_a_per_frame_difference():
    policy = load_policy()
    policy.build_observation((0.0, 0.0, 0.0), (0.0, 0.0, 2.0))
    observation = policy.build_observation((0.0, 0.0, 0.0), (0.1, 0.0, 2.0))
    assert observation[12].item() == pytest.approx(0.1)
