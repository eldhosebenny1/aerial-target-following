# Publishing checklist

Practical notes for making this repository legible to a robotics engineer, a
researcher or a recruiter, without overstating what exists.

## Name and description

Repository name options, in order of preference:

| Name | Why |
| --- | --- |
| `aerial-target-following` | Describes the actual capability; reads as a research repo |
| `monocular-target-following-px4` | Emphasises the monocular constraint and the autopilot |
| `perception-rl-navigation` | Broader, better if the project grows beyond one target class |

Description (GitHub allows 350 characters):

> Monocular target following and obstacle avoidance for a PX4 multirotor. YOLO +
> ByteTrack perception, pinhole range estimation, Depth Anything V2 safety override,
> and a TorchScript reinforcement-learning controller commanded over MAVSDK offboard.

Suggested topics: `robotics`, `computer-vision`, `reinforcement-learning`, `px4`,
`mavsdk`, `object-detection`, `depth-estimation`, `autonomous-navigation`, `drones`,
`pytorch`.

Keep the description about perception, estimation and control. That is what the code
does, and it is what a reviewer will evaluate.

## Badges

Only add badges you can defend:

- Python version, PyTorch, PX4/MAVSDK, license: safe, they describe the stack.
- A CI badge: add only once `.github/workflows/` actually runs the test suite. A
  passing badge for tests that do not exist is the fastest way to lose credibility.
- Downloads/stars badges: skip.

## Visuals

The README reads better with three images, in this order:

1. **Architecture diagram.** The Mermaid block in the README renders on GitHub. If you
   prefer a hand-drawn or Figma version, export it to `docs/assets/architecture.png`
   and reference that instead.
2. **GUI screenshot.** One frame with the target box, track id, range, FPS and the
   avoidance label visible, plus the depth window next to it. Crop, do not scale up.
3. **Demo clip.** 15-30 s: target acquired, follow, obstacle override, land. Host it on
   YouTube or attach it to a Release and link it. Do not commit an MP4; Git history is
   permanent and video bloat is not.

Put images in `docs/assets/`. Reference them from the README with relative paths.

## Results to show

Fill this in only with numbers you actually measured. If a row has no data yet, delete
the row rather than inventing one.

| Metric | How it was measured | Value |
| --- | --- | --- |
| Policy forward pass | 500 iterations, CPU | 0.23 ms |
| Detector latency | per-frame timing on the flight computer | not measured |
| Depth latency | per-frame timing on the flight computer | not measured |
| End-to-end loop rate | frames per second from the overlay | not measured |
| Range error vs tape measure | 1-10 m, five points | not measured |
| Tracking continuity | longest run without an id switch | not measured |
| Flight duration / outcome | flight log | not measured |

If you add training curves, export the TensorBoard scalars as a PNG into `docs/assets/`
and keep the event files out of Git.

## What to upload

| Include | Notes |
| --- | --- |
| Source code (`src/`) | The whole point |
| Tests | Proves the pure logic is checked |
| `requirements*.txt` | Exact import set, no invented packages |
| `README.md` and `docs/` | Architecture, review, this checklist |
| `models/policy.pt` | 19.5 KB, makes inference runnable |
| Training configs | Once the training code is published |
| Small example assets | A short clip extracted as a few PNG frames is fine |
| Evaluation results | CSV or Markdown tables, plus plots |

| Exclude | Reason |
| --- | --- |
| `best.pt` and other large weights | Use Releases, LFS or external storage |
| `model_*.pt`, `checkpoint_*.pt` | Training by-products, reproducible or not needed |
| TensorBoard event files | Binary, large, no value in history |
| Datasets and raw video | Size, licensing, privacy |
| Generated footage and screenshots of every run | Bloat; pick the best three |
| `__pycache__`, `.venv`, IDE folders | Already in `.gitignore` |
| `.env`, API keys, licence keys, SSH material | Never; assume any committed secret is public |
| Absolute local paths | Nothing breaks a clone faster than `C:\Users\...` hardcoded |

Before the first push:

```bash
git status --ignored --short | head -40   # confirm what is and is not tracked
git ls-files | xargs -I{} du -h {} | sort -h | tail -10   # biggest tracked files
```

If a file over 5 MB slips in, remove it from the index (`git rm --cached`) before
committing. If it reaches a pushed commit, the history needs rewriting, which is
painful on a public repo.

## Commit structure

One large "initial commit" is acceptable for a project that already exists, and that
is what this repository starts with. From here, keep commits small and scoped:

```
perception: lock target by track id instead of first box
control: make acceleration limit time-based
docs: add flight log format and sample
```

Group by subsystem, not by file. Avoid a series of "update", "fix", "final" commits;
they cost nothing to make and tell a reviewer nothing.

If you want the history to show the engineering process, a defensible sequence is:

1. `initial commit: perception + policy + offboard control` (this state)
2. `perception: add depth-based avoidance override`
3. `tests: cover distance, limiter and avoidance logic`
4. `docs: architecture and code review`
5. one commit per item you fix from `docs/CODE_REVIEW.md`

## Honesty checklist

Before publishing, make sure every claim in the README has one of these tags, either
explicitly or by context:

- **implemented** — the code exists and runs
- **trained** — weights exist, but the training setup may not be in the repo
- **tested** — a test or a replay covers it
- **simulated** — SITL only
- **experimental** — tried, not validated
- **future work** — not built

Anything else should say "unknown". A reviewer will forgive a gap that is named. They
will not forgive a claim that collapses under one question.
