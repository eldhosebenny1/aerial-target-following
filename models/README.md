# Models

Two weight files are used at runtime. Only one of them is small enough to keep
in Git.

| File | Role | In this repo |
| --- | --- | --- |
| `policy.pt` | TorchScript actor exported from the RL training run | Yes, ~19.5 KB |
| `best.pt` | Ultralytics detector for the tracked target class | No |

`src/utils/config.py` resolves both paths and allows overrides:

```bash
export MODEL_PATH=/abs/path/to/best.pt      # or set it in config.py
export POLICY_PATH=models/policy.pt
```

## policy.pt

Exported with a TorchScript policy exporter (`_TorchPolicyExporter`), so it
loads with `torch.jit.load` and carries no training code. Inspection shows:

```
actor:        Linear(15 -> 32) -> ELU -> Linear(32 -> 32) -> ELU -> Linear(32 -> 4)
normalizer:   EmpiricalNormalization, mean/var over 15 observation dimensions
```

The normalizer is inside the exported module, so observations must be passed in
the same unscaled form the training environment used. The 15-dim layout the
control loop builds is documented in the main README; the two must stay in sync.

## best.pt

A trained Ultralytics detection checkpoint for a single target class. Drop it in
`models/` or point `MODEL_PATH` at it. The detector is built from the weights alone, so
any single-class Ultralytics model loads without a config file.

## Committing weights

- `policy.pt` is ~20 KB. Committing it is reasonable: it makes the inference
  path runnable without a download step.
- `best.pt` is usually tens of MB. Do not commit it. Use one of:
  - **GitHub Releases** (recommended): attach the file to a version tag and
    document the download command in this file.
  - **Git LFS**: `git lfs track "*.pt"`, add `.gitattributes`, then commit. Note
    that LFS has storage and bandwidth quotas and clones without `git-lfs`
    installed silently get pointer files.
  - **External storage** (Hugging Face Hub, S3, institutional drive) with a
    checksum and download instructions.
- Never commit `checkpoint_*.pt`, `model_*.pt` snapshots or TensorBoard event
  files. Those are training by-products, not releases.

Whichever option you pick, record the file size and a SHA-256 checksum here so a
reviewer can confirm they have the weights the code was evaluated with.
