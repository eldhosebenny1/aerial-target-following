# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Ego drone chasing a scripted enemy in Isaac Lab.

The ego aircraft is a Crazyflie with no rotor model: the policy commands a
forward/lateral velocity and a thrust magnitude, and those are turned into a force
on the root body. The enemy is not learned -- it flies a pursuit rule toward the ego
with a bit of noise, which is enough to give the policy something to react to.
"""

import torch
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.markers import CUBOID_MARKER_CFG, VisualizationMarkers
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import euler_xyz_from_quat, quat_apply

from .drone_combat_env_cfg import DroneCombatEnvCfg


class DroneCombatEnv(DirectRLEnv):
    cfg: DroneCombatEnvCfg

    def __init__(self, cfg: DroneCombatEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.ego = self.scene["robot"]
        self.enemy = self.scene["enemy"]

        self.ego_root_idx = 0
        self.enemy_root_idx = 0
        self.action_scale = self.cfg.action_scale

        self.enemy_marker = None
        self.set_debug_vis(self.cfg.debug_vis)

    def set_up_scene(self, scene):
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(size=(200.0, 200.0), color=(0.5, 0.5, 0.5)),
        )

        # Two lights: a dome so nothing is pitch black, plus a sun for shadows.
        dome_cfg = sim_utils.DomeLightCfg(intensity=5000.0, color=(0.75, 0.75, 0.75))
        dome_cfg.func("/World/Light/Dome", dome_cfg)

        sun_cfg = sim_utils.DistantLightCfg(intensity=3000.0, color=(1.0, 0.96, 0.9))
        sun_cfg.func("/World/Light/Sun", sun_cfg)

        scene.add(self.cfg.robot, name="robot")
        scene.add(self.cfg.enemy, name="enemy")

        if self.device == "cpu":
            scene.filter_collisions(global_prim_paths=[])

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clone().to(self.device)

        num_envs = self.num_envs
        num_bodies = self.ego.num_bodies

        # Ego: thrust along body -z, plus a proportional kick on the horizontal
        # velocity error. The policy never sees the force level, only the command.
        vel_cmd = self.actions[:, :2] * self.action_scale
        thrust_mag = self.actions[:, 2].clamp(min=0.0, max=1.0) * 40.0

        body_quat = self.ego.data.root_state_w[:, 3:7]
        thrust_local = torch.zeros((num_envs, 3), device=self.device)
        thrust_local[:, 2] = -thrust_mag
        thrust_world = quat_apply(body_quat, thrust_local)

        current_vel = self.ego.data.root_state_w[:, 7:10]
        lateral_error = vel_cmd - current_vel[:, :2]
        lateral_force = lateral_error * 6.0

        combined_force = thrust_world + torch.cat(
            [lateral_force, torch.zeros((num_envs, 1), device=self.device)], dim=-1
        )

        ego_forces = torch.zeros((num_envs, num_bodies, 3), device=self.device)
        ego_forces[:, self.ego_root_idx] = combined_force
        ego_forces = torch.nan_to_num(ego_forces, nan=0.0, posinf=0.0, neginf=0.0)
        ego_forces = torch.clamp(ego_forces, min=-80.0, max=80.0)

        self.ego.set_external_force_and_torque(ego_forces, torch.zeros_like(ego_forces))

        # Enemy: fly at the ego, flip to backing away once it is close, and jitter
        # so the policy cannot just sit in one pose.
        ego_pos = self.ego.data.root_state_w[:, :3]
        enemy_pos = self.enemy.data.root_state_w[:, :3]

        rel = ego_pos - enemy_pos
        dist = torch.norm(rel, dim=-1)
        direction = rel / (dist.unsqueeze(-1) + 1e-6)

        speed = torch.where(dist < 15.0, -25.0, 20.0).unsqueeze(-1)
        enemy_vel_cmd = direction * speed
        enemy_vel_cmd += torch.randn_like(enemy_vel_cmd) * 6.0

        current_vel_enemy = self.enemy.data.root_state_w[:, 7:10]
        vel_error_enemy = enemy_vel_cmd - current_vel_enemy
        enemy_force = vel_error_enemy * 10.0

        enemy_forces = torch.zeros((num_envs, num_bodies, 3), device=self.device)
        enemy_forces[:, self.enemy_root_idx] = enemy_force
        enemy_forces = torch.nan_to_num(enemy_forces, nan=0.0, posinf=0.0, neginf=0.0)
        enemy_forces = torch.clamp(enemy_forces, min=-80.0, max=80.0)

        self.enemy.set_external_force_and_torque(enemy_forces, torch.zeros_like(enemy_forces))

    def _get_observations(self):
        ego_root = self.ego.data.root_state_w
        enemy_root = self.enemy.data.root_state_w

        ego_pos = ego_root[:, :3]
        ego_quat = ego_root[:, 3:7]
        ego_linvel = ego_root[:, 7:10]
        ego_angvel = ego_root[:, 10:13]

        enemy_pos = enemy_root[:, :3]
        enemy_linvel = enemy_root[:, 7:10]

        rel_pos = enemy_pos - ego_pos
        rel_vel = enemy_linvel - ego_linvel
        dist = torch.norm(rel_pos, dim=-1, keepdim=True)

        # Heading error is wrapped into [-pi, pi] by atan2 difference, which is fine
        # as long as the policy does not need continuity across the +/-pi boundary.
        _, _, ego_yaw = euler_xyz_from_quat(ego_quat)
        rel_heading = torch.atan2(rel_pos[:, 1], rel_pos[:, 0]) - ego_yaw
        rel_heading = rel_heading.unsqueeze(-1)

        obs = torch.cat(
            [ego_pos, ego_quat, ego_linvel, ego_angvel, rel_pos, rel_vel, dist, rel_heading],
            dim=-1,
        )
        obs = torch.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        obs = torch.clamp(obs, min=-100.0, max=100.0)

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        ego_pos = self.ego.data.root_pos_w
        ego_quat = self.ego.data.root_quat_w
        ego_vel = self.ego.data.root_lin_vel_w
        ego_angvel = self.ego.data.root_ang_vel_w
        enemy_pos = self.enemy.data.root_pos_w

        rel_pos = enemy_pos - ego_pos
        dist = torch.norm(rel_pos, dim=-1)

        # Pursuit: reward velocity that points at the enemy, but only while there is
        # still distance to close and the aircraft is off the floor.
        direction = rel_pos / (dist.unsqueeze(-1) + 1e-6)
        vel_norm = torch.norm(ego_vel, dim=-1)
        alignment = torch.sum(ego_vel * direction, dim=-1) / (vel_norm + 1e-6)
        flying_high = (ego_pos[:, 2] > 0.8).float()
        pursuit = self.cfg.rew_scale_pursuit * alignment * (dist > 8.0).float() * flying_high

        alive = self.cfg.rew_scale_alive * (~self.reset_buf).float()
        dist_pen = self.cfg.rew_scale_dist * dist
        energy_pen = self.cfg.rew_scale_energy * torch.norm(self.actions[:, :3], dim=-1)

        # Attitude terms dominate the sum. Without them the policy learns to roll
        # over and thrust sideways, which reaches the enemy fastest and looks absurd.
        roll, pitch, _ = euler_xyz_from_quat(ego_quat)
        upright_raw = 1.0 - (torch.abs(roll) + torch.abs(pitch))
        upright_bonus = torch.clamp(self.cfg.rew_scale_upright * upright_raw**2, min=0.0, max=8.0)

        angvel_norm = torch.norm(ego_angvel, dim=-1)
        angvel_stability = self.cfg.rew_scale_angvel * torch.exp(-angvel_norm * 3.0)

        # Hold roughly 1.5 m of altitude.
        height_error = ego_pos[:, 2] - 1.5
        height_bonus = self.cfg.rew_scale_height * torch.exp(-(height_error**2) * 2.0)

        low_height_pen = -30.0 * (ego_pos[:, 2] < 0.3).float()
        crash_pen = -80.0 * (ego_pos[:, 2] < 0.1).float()

        total_reward = (
            alive * 1.2
            + pursuit * 1.0
            + dist_pen * 0.8
            + energy_pen * 1.0
            + upright_bonus * 2.5
            + angvel_stability * 2.0
            + height_bonus * 1.5
            + low_height_pen
            + crash_pen
        )

        total_reward = torch.nan_to_num(total_reward, nan=0.0, posinf=0.0, neginf=0.0)
        return torch.clamp(total_reward, min=-50.0, max=50.0)

    def _get_dones(self):
        dist = torch.norm(
            self.enemy.data.root_state_w[:, :3] - self.ego.data.root_state_w[:, :3], dim=-1
        )

        terminated = (dist < 3.0) | (self.episode_length_buf >= self.max_episode_length - 1)
        truncated = torch.zeros_like(terminated, dtype=torch.bool)

        return terminated, truncated

    def _reset_idx(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        super()._reset_idx(env_ids)

        num_reset = len(env_ids)
        pos_range = 30.0
        offset = torch.rand((num_reset, 3), device=self.device) * pos_range - pos_range / 2

        ego_default = self.ego.data.default_root_state[env_ids].clone()
        ego_default[:, :3] += offset
        self.ego.write_root_state_to_sim(ego_default, env_ids=env_ids)

        # Start the enemy 8 m out on both axes, with the offset so the pair keeps
        # its relative geometry inside whichever patch of the plane it lands on.
        enemy_default = self.enemy.data.default_root_state[env_ids].clone()
        enemy_default[:, :3] += offset + torch.tensor([8.0, 8.0, 1.5], device=self.device)
        self.enemy.write_root_state_to_sim(enemy_default, env_ids=env_ids)

        zero_vel = torch.zeros((num_reset, 6), device=self.device)
        self.ego.write_root_velocity_to_sim(zero_vel, env_ids=env_ids)
        self.enemy.write_root_velocity_to_sim(zero_vel, env_ids=env_ids)

    def _apply_action(self) -> None:
        pass

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "enemy_marker"):
                marker_cfg = CUBOID_MARKER_CFG.copy()
                marker_cfg.markers["cuboid"].size = (0.15, 0.15, 0.15)
                marker_cfg.prim_path = "/Visuals/EnemyPos"
                self.enemy_marker = VisualizationMarkers(marker_cfg)
            self.enemy_marker.set_visibility(True)
        else:
            if hasattr(self, "enemy_marker"):
                self.enemy_marker.set_visibility(False)

    def _debug_vis_callback(self):
        if hasattr(self, "enemy_marker") and self.enemy_marker is not None:
            enemy_pos = self.enemy.data.root_pos_w.clone()
            enemy_pos[:, 2] += 0.5
            self.enemy_marker.visualize(enemy_pos)
