# Training

The reinforcement-learning training code is **not in this repository**. What
exists here is the result of that training: `models/policy.pt`, a TorchScript
actor exported from the run.

That gap matters, so this file states plainly what is known and what is not.

## Known

- The exported module is a `_TorchPolicyExporter` wrapper with an
  `EmpiricalNormalization` module and a 32-32 ELU actor, which is the shape
  produced by stable-baselines3-style PPO exports.
- The actor takes 15 observation dimensions and returns 4 action dimensions.
- Training snapshots were saved periodically (`model_0.pt`, `model_50.pt`,
  `model_100.pt`, `model_150.pt`) and TensorBoard event files were written.
- The control loop observes `[ned velocity, 3 unused dims, gravity vector,
  target error, error rate]` and consumes `[vx, vy, _, yaw rate]`.

## Unknown

- Environment, reward function, episode termination and curriculum.
- PPO hyperparameters, total timesteps, seed.
- Observation and action normalisation statistics of the *training* env beyond
  what is baked into the exported normalizer.
- Whether the observation vector in the flight code matches the training env
  dimension for dimension. It has the right *length*, but the frame conventions
  and the three zero-filled dimensions are unverified. See
  `docs/CODE_REVIEW.md`.

Fill these in before describing the policy as validated, because a policy that
receives a shifted observation can still produce smooth, plausible-looking
commands while being wrong.

## Layout

```
training/
├── configs/          # env and PPO configs, once the training code is published
├── checkpoints/      # local snapshots, git-ignored
└── README.md
```

`configs/` and `checkpoints/` are placeholders. `checkpoints/` is ignored by
`.gitignore` except for the `.gitkeep` file.

## Publishing the training code

If the training project can be open-sourced, add:

1. The environment definition, including the exact observation construction and
   reward terms.
2. The training entry point and config file.
3. An export script that produces `policy.pt` (the same TorchScript wrapper used
   at inference time).
4. A short note in the main README replacing "not included" with what is now
   reproducible.

Keep checkpoints and TensorBoard logs out of Git. A single final checkpoint in a
Release, plus the configs and the tensorboard curves as images in `docs/`, is
enough for someone to judge the training run.
