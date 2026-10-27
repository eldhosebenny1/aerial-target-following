import numpy as np
import torch

from src.utils import config

# Body-frame direction of gravity, as fed to the policy during training.
GRAVITY_BODY = torch.tensor([0.0, 0.0, -1.0])


class TargetPolicy:
    """TorchScript actor plus the 15-dim observation layout it was trained on.

    Layout: [ned_velocity * 0.1 (3), unused (3), gravity (3),
             target_error * 0.2 (3), error_rate * 0.1 (3)]
    """

    def __init__(self, path=config.POLICY_PATH, device="cpu",
                 max_speed=config.MAX_SPEED, max_yaw=config.MAX_YAW):
        try:
            self.policy = torch.jit.load(path, map_location=device)
        except Exception:
            self.policy = torch.load(path, map_location=device)
        self.policy.eval()

        self.device = device
        self.max_speed = max_speed
        self.max_yaw = max_yaw
        self.last_error = torch.zeros(3)
        self.observation_size = 15

    def build_observation(self, velocity_ned, position_error):
        velocity = torch.as_tensor(velocity_ned, dtype=torch.float32)
        error = torch.as_tensor(position_error, dtype=torch.float32)

        error_rate = (error - self.last_error) * 10.0
        self.last_error = error

        return torch.cat([
            velocity * 0.1,
            torch.zeros(3) * 0.1,
            GRAVITY_BODY,
            error * 0.2,
            error_rate * 0.1,
        ], dim=-1)

    def act(self, velocity_ned, position_error):
        """Returns (vx, vy, yaw_rate) in m/s and deg/s for the offboard setpoint."""
        observation = self.build_observation(velocity_ned, position_error)
        observation = observation.unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.policy(observation).cpu().numpy().flatten()

        vx = float(np.clip(action[0] * self.max_speed, -self.max_speed, self.max_speed))
        vy = float(np.clip(action[1] * self.max_speed, -self.max_speed, self.max_speed))
        yaw = float(np.clip(action[3] * self.max_yaw, -self.max_yaw, self.max_yaw))
        return vx, vy, yaw
