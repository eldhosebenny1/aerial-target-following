# Training (Isaac Lab)

The two files in `isaaclab/` are not a standalone Python package. They are an Isaac Lab
task, and they only run inside an Isaac Lab installation, which needs Isaac Sim and an
NVIDIA GPU. Nothing here is imported by the flight stack in `src/`.

| File | Role |
| --- | --- |
| `isaaclab/drone_combat_env.py` | `DirectRLEnv` subclass: physics stepping, reward, resets |
| `isaaclab/drone_combat_env_cfg.py` | Task config: scene, drones, reward scales, action scale |

The module imports `from .drone_combat_env_cfg import DroneCombatEnvCfg`, so the two
files have to sit next to each other with these names. If you rename the config file,
update that import.

## The task

Two Crazyflie cf2x bodies on a 200 m plane, 256 copies in parallel. There is no rotor
model: the policy commands a horizontal velocity and a thrust magnitude, which are
converted into a single external force on the root body. The second aircraft is not
learned, it flies a pursuit rule at the first one with noise added, and switches to
backing away inside 15 m. An episode ends when the two are within 3 m, or after 10 s.

Observations are 21 numbers: ego position, quaternion, linear velocity, angular
velocity, relative position, relative velocity, distance, heading error. Actions are
3 numbers: `vx` and `vy` in the body frame, scaled by `action_scale`, plus thrust.
The reward is a pursuit term on velocity alignment, an alive term, distance and energy
penalties, and three larger terms for staying upright, staying still in rotation and
holding 1.5 m of altitude.

## Getting it running

You need Isaac Lab installed first, following their setup instructions. Then pick one
of the two routes below.

### Route 1: paste into the built-in quadcopter task (recommended)

Isaac Lab already ships a drone environment at
`source/isaaclab_tasks/isaaclab_tasks/direct/quadcopter/`. It contains a working task
registration and the RL agent configs, so reusing that folder is less work than
starting a new package: you get the `agents/` YAML files without writing them.

1. Copy the two files into that folder:

   ```bash
   cp training/isaaclab/drone_combat_env.py \
      <isaaclab>/source/isaaclab_tasks/isaaclab_tasks/direct/quadcopter/
   cp training/isaaclab/drone_combat_env_cfg.py \
      <isaaclab>/source/isaaclab_tasks/isaaclab_tasks/direct/quadcopter/
   ```

2. Open that folder's `__init__.py` and add a registration next to the existing one.
   The agent entries point at the YAML files already in the folder:

   ```python
   import gymnasium as gym

   from . import agents
   from .drone_combat_env import DroneCombatEnv
   from .drone_combat_env_cfg import DroneCombatEnvCfg

   gym.register(
       id="Isaac-DroneCombat-Direct-v0",
       entry_point=f"{__name__}.drone_combat_env:DroneCombatEnv",
       disable_env_checker=True,
       kwargs={
           "env_cfg_entry_point": f"{__name__}.drone_combat_env_cfg:DroneCombatEnvCfg",
           "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
           "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
       },
   )
   ```

   Isaac Lab installs `isaaclab_tasks` in editable mode, so no reinstall is needed after
   editing that file.

3. Train, from the Isaac Lab root:

   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
       --task Isaac-DroneCombat-Direct-v0 \
       --algorithm PPO \
       --num_envs 256 \
       --headless
   ```

   or with rl_games:

   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py \
       --task Isaac-DroneCombat-Direct-v0 \
       --num_envs 256 \
       --headless
   ```

4. Watch the result:

   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py \
       --task Isaac-DroneCombat-Direct-v0 --num_envs 16
   ```

Run without `--headless` for the first few thousand steps to confirm the drones spawn
above the plane and the marker follows the second aircraft.

### Route 2: a separate project

If you would rather not modify your Isaac Lab checkout, use the project template
generator in the Isaac Lab docs (`./isaaclab.sh --new`), choose a Direct RL
environment, and replace the generated env and cfg files with these two. Then add the
same `gym.register` block to the generated `__init__.py` and add your own `agents/`
YAML files, since a new project does not come with the quadcopter agent configs.

## Porting notes

These matter if you train here and then fly the result.

- **The observation layouts do not match.** This environment produces 21 observations
  and 3 actions. The actor in `models/policy.pt` is exported as 15 in, 4 out, and
  `src/navigation/policy.py` builds that 15-dim vector. A policy trained here cannot be
  dropped into `models/` without changing the observation construction on the flight
  side and re-exporting.
- **Different action meaning.** In this environment the horizontal channels are a
  velocity command with a force controller behind them, and thrust is commanded
  directly. On the aircraft, `src/` sends body-frame velocity setpoints to PX4 and lets
  the autopilot run its own attitude and thrust loops.
- **Two declarations were wrong as received and are corrected here.** The cfg declared
  `action_space = 6` and `observation_space = 32`, while `_pre_physics_step` reads
  three action channels and `_get_observations` concatenates 21 values. The dimensions
  now match the code. Any checkpoint trained against the old numbers has the wrong
  input and output width for these files.
- **Reward weights now come from the config.** `rew_scale_upright`, `rew_scale_angvel`
  and `rew_scale_height` were declared but the reward function used literals; the
  values are unchanged (8.0, 5.0, 4.0), so behaviour is identical and the config
  actually controls the shaping.
- **The episode time limit is reported as termination, not truncation.** `_get_dones`
  folds `episode_length_buf >= max_episode_length - 1` into `terminated`, so the value
  bootstrap is skipped at the 10 s cut-off. Worth splitting into `truncated` if you
  care about the last few steps of each episode.
- **`--num_envs` and `scene.num_envs` are separate.** The CLI flag overrides the config
  value at runtime; the 256 in the cfg is only the default.

## Files and logs

Isaac Lab writes runs to `logs/`, which is ignored by `.gitignore`, along with
TensorBoard events and video captures. Training snapshots are `*.pt` and are ignored
too, for the same reason described in `models/README.md`: keep them in a Release or
outside Git, not in the repository history.
