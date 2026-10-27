# Examples

Offline, runnable snippets that do not need a drone.

| Script | What it shows |
| --- | --- |
| `demo_avoidance.py` | Feeds a synthetic depth sequence through the obstacle-avoidance state machine and the velocity limiter, printing the command transition (CLEAR -> SLIDE/VEER/SLOW -> CLEAR) and saving an annotated depth frame to `outputs/`. |

```bash
python examples/demo_avoidance.py
```

Changing `DANGER_THRESHOLD` or `BUFFER_SIZE` in `src/utils/config.py` and
re-running is the fastest way to see how the majority filter affects when the
override engages.

Screenshots and short screen recordings that you want to show on GitHub belong
in `docs/assets/`, not here. Keep generated output out of the repository.
