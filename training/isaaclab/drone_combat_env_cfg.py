# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the drone pursuit task."""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import PhysxCfg, SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


@configclass
class DroneCombatEnvCfg(DirectRLEnvCfg):
    """Pursuit task built on two Crazyflie cf2x bodies."""

    decimation = 2
    episode_length_s = 10.0

    # The policy outputs [vx_cmd, vy_cmd, thrust] and receives 21 numbers:
    # ego pos 3, ego quat 4, ego lin vel 3, ego ang vel 3, relative pos 3,
    # relative vel 3, distance 1, heading error 1.
    action_space = 3
    observation_space = 21
    state_space = 0

    debug_vis = True

    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120.0,
        render_interval=decimation,
        physx=PhysxCfg(
            solver_type=1,
            max_position_iteration_count=8,
            max_velocity_iteration_count=2,
        ),
        gravity=(0.0, 0.0, -9.81),
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256,  # 64 or 128 if VRAM runs out
        env_spacing=8.0,
        replicate_physics=True,
    )

    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Crazyflie/cf2x.usd",
            scale=(2.5, 2.5, 2.5),  # the stock model is tiny at these spawn distances
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_linear_velocity=30.0,
                max_angular_velocity=30.0,
                max_depenetration_velocity=5.0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.5),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={},
    )

    enemy: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Enemy",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Crazyflie/cf2x.usd",
            scale=(2.5, 2.5, 2.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_linear_velocity=30.0,
                max_angular_velocity=30.0,
                max_depenetration_velocity=5.0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(4.0, 4.0, 1.5),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={},
    )

    # Reward scales. The three attitude terms are the ones that keep the aircraft
    # flying upright; raise them before raising the pursuit term if it flips.
    rew_scale_alive = 1.2
    rew_scale_pursuit = 15.0
    rew_scale_dist = -0.03
    rew_scale_energy = -0.01
    rew_scale_upright = 8.0
    rew_scale_angvel = 5.0
    rew_scale_height = 4.0

    action_scale = 40.0
