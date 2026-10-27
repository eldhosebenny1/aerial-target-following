# Code review

This is a review of the flight code as it was written, not of an imagined better
system. Each item states what the code does now, and separately what I would change.
Nothing in the list below has been applied unless it says so explicitly.

Severity is about consequences on a real aircraft, not code style.

| # | Finding | Severity | State |
| --- | --- | --- | --- |
| 1 | Detector path constant never reaches `YOLO()` | crash | fixed in refactor |
| 2 | Depth model was fed the annotated frame (bounding boxes burned in) | correctness | fixed in refactor |
| 3 | NED velocity mixed with body/camera-frame error in one observation | high risk | open |
| 4 | Blocking inference can stall the offboard setpoint stream | safety | open |
| 5 | No RC override, geofence, setpoint watchdog or altitude check | safety | open |
| 6 | `MAX_ACCEL` is per loop iteration, not per second | control | open |
| 7 | Yaw rate is clipped but not rate-limited; jumps to zero on override | control | open |
| 8 | Target is always box 0; track id never used for selection | robustness | open |
| 9 | Lost target means hover forever, no timeout, no search | robustness | open |
| 10 | Error history goes stale while the avoidance override is active | control | open |
| 11 | Depth threshold applied to per-frame min-max normalised values | calibration | open |
| 12 | `VEER LEFT` sends a positive `vy`, which is right in body FRD | likely sign error | open |
| 13 | Focal length assumed to match the rotated frame; capture size not read back | calibration | open |
| 14 | Range assumes a fixed target width, no truncation handling | accuracy | open |
| 15 | One telemetry stream opened per iteration | efficiency | open |
| 16 | Fixed 10 s wait after takeoff, no armability or altitude check | safety | open |
| 17 | Three zero-filled observation dimensions with no documented role | correctness | open |
| 18 | `action[2]` ignored, vertical control delegated to PX4 altitude hold | design | documented |
| 19 | Window close button does not end the loop, only `q` | usability | open |
| 20 | No logging of frames, setpoints or telemetry | verification | open |

## Changes made during the refactor

Structural only, except where listed here. These are deliberate and disclosed:

1. **Detector path (item 1).** The prototype declared
   `enemy_detecting_MODEL_PATH = "your models path"` and then called
   `YOLO(MODEL_PATH)`, which raises `NameError` at import time. The refactor reads
   `config.MODEL_PATH`, overridable via the `MODEL_PATH` environment variable.
   Without this the original file cannot run at all.
2. **Depth input (item 2).** The prototype drew the target box and label onto the
   frame and then resized *that* frame for the depth model, so every inference saw a
   green rectangle and text burned into the image. The refactor keeps a separate
   canvas for drawing and runs depth on the clean frame. This changes the depth
   numbers slightly and therefore the avoidance decisions.
3. **Offboard stop logging.** `offboard.stop()` failures are printed instead of
   silently swallowed. No behavioural change beyond the log line.

Everything else was preserved on purpose, including the odd parts: the zero-filled
observation dimensions, the ignored action index, first-box target selection, the
`* 10` error-rate gain and the per-iteration acceleration clamp.

## Critical

### 3. Frame conventions are mixed inside one observation

**Current.** The observation concatenates:

- `telemetry.velocity_ned()` — north/east/down, an earth-fixed frame;
- a gravity vector `[0, 0, -1]`, which only makes sense in a body frame;
- the target error `[lateral, 0, forward]`, where forward is the camera optical axis
  and lateral is horizontal-right in image space.

The command side then sends `VelocityBodyYawspeed`, which is body FRD. So the loop
mixes earth-fixed, body and camera conventions without conversion. It only works if
the training environment did exactly the same thing.

**Why it matters.** A policy trained on body-frame velocity, fed NED velocity, will
see a rotated input whenever the aircraft yaws. Behaviour degrades in a way that
looks like bad tuning rather than a data bug, because the commands remain smooth and
plausible.

**Recommended.** Decide on one convention (body FRD is the natural choice, since the
commands are body FRD), convert at the boundary, and check the observation vector
against a hand-computed example from the training environment. The README flags it as
the first thing to confirm if behaviour looks wrong.

### 4. Blocking inference and the offboard setpoint stream

**Current.** Detection, depth estimation and tracking all run inline in the asyncio
task. MAVSDK connection handling and telemetry share that event loop. A slow frame
therefore delays both the setpoint and the telemetry that feeds the next observation.

**Why it matters.** PX4 drops out of offboard when setpoints stop arriving, and the
fallback depends on the failsafe configuration. There is no watchdog in this code and
no check that the previous setpoint was actually delivered.

**Recommended.** Measure the loop period and log it. If it can exceed the offboard
timeout, move inference to a worker thread or process and keep only command
arbitration plus a fixed-rate setpoint publisher (20-50 Hz) in the loop, with
`asyncio.wait_for` around MAVSDK calls. Handle the `offboard.stop` / mode-change case
explicitly.

### 5. No independent safety path

**Current.** The only automatic responses are `offboard.start()` failure (land) and
the `finally` block (stop offboard, land). There is no RC override, no geofence, no
battery threshold, no minimum altitude, no maximum distance from home and no
setpoint-age check. A `SIGKILL` or a power loss on the companion computer leaves the
aircraft in its last commanded mode.

**Recommended.** Keep PX4's own failsafes configured and tested (RC loss, data link
loss, offboard timeout, battery, geofence) and document the exact parameter values.
Treat the Python code as a command source, not as a safety system. Add a mode check
before every setpoint and stop commanding the moment the vehicle leaves offboard.

## High

### 6. Acceleration limit is not time-based

**Current.** `MAX_ACCEL = 0.08` is applied per loop iteration. The loop rate is set by
inference, so the same constant means 0.8 m/s^2 at 10 Hz and 2.0 m/s^2 at 25 Hz.

**Recommended.** Convert to a rate: `max_delta = max_accel_m_s2 * dt`, clamp `dt` to a
sane range, and pick `max_accel_m_s2` from airframe limits.

### 7. Yaw is not rate-limited

**Current.** `target_yaw` is clipped to +-15 deg/s but never slewed, and the override
branch sets it to zero instantly. A one-iteration jump between -15 and +15 deg/s is a
15 deg/s step every control period.

**Recommended.** Add a yaw-axis limiter (`max_yaw_accel`) next to `VelocityLimiter`,
or use a first-order filter on the yaw rate.

### 8. Target selection ignores the track id

**Current.** ByteTrack assigns ids, and the id is drawn on screen, but the loop always
uses `boxes.xyxy[0]`. With two objects of the same class, the commanded target can
change between frames without any lost-detection event.

**Recommended.** Lock onto an id on first acquisition and follow it until it is lost
for N frames. Expose the lock state in the overlay, and log switches.

### 9. Lost target behaviour

**Current.** If no target is visible, the setpoint is zeroed. The aircraft hovers until
the battery or the pilot ends the flight. There is no timeout, no search yaw, no
"return to last seen position".

**Recommended.** Pick an explicit policy: hover with a timeout, then a slow yaw scan,
then land (or RTL). Whichever you choose, implement it deliberately and document it,
because hovering indefinitely is a decision too.

### 10. Error history goes stale across overrides

**Current.** `policy.last_error` is only updated when the policy runs. During an
avoidance override the policy is skipped, so when control returns the rate term is one
large jump. The first call after start-up has the same problem against a
zero-initialised history, where the rate term starts at 2.0 for a 2 m error.

**Recommended.** Update the history on every frame, or reset it deliberately on
re-entry and add a warm-up period. Do not leave it implicitly stale.

### 11. Depth threshold on a per-frame normalised map

**Current.** Depth Anything outputs relative inverse depth. Each frame is min-max
normalised, then the centre-third mean is compared with `DANGER_THRESHOLD = 0.355`.
The value has 16 significant digits, which suggests it came out of a tuning sweep, and
it is tuned to the recordings that sweep used.

**Why it matters.** Min-max normalisation is scene-dependent. A frame with one very
close object compresses everything else; a flat frame stretches noise across the full
range. The same obstacle can produce different decisions in different scenes.

**Recommended.** Normalise against a running reference (percentiles over a window), or
move to a metric depth model (Depth Anything V2 metric variants, or a stereo/ToF
sensor) and set the threshold in metres. Keep the recordings used to pick the number
with the config so it can be re-tuned.

### 12. Lateral direction of the avoidance manoeuvre

**Current.** For an obstacle on the right (`left_score < right_score`) the code returns
`VEER LEFT` with `vy = +0.5`. In MAVSDK's body frame, positive `y` is to the right. So
the label and the command disagree, and the aircraft would manoeuvre *toward* the
obstacle it just detected.

**Recommended.** Confirm the sign on the airframe (or in SITL with a HITL-style check)
and make the label follow the command. This is a one-character fix with a large
consequence, so it belongs in a test: a synthetic depth map with the obstacle on the
right must produce a leftward (negative) `vy`.

### 13. Focal length versus rotated frame

**Current.** The capture is requested at 1280x720 and then rotated 90 degrees. The
distance model uses `FOCAL_LENGTH_PX = 945` for the rotated image. `CAP_PROP_FRAME_WIDTH`
is a request, not a guarantee; many cameras ignore it and deliver something else, and
the rotation swaps the axes.

**Recommended.** Read back the actual resolution after opening the device, assert it,
and derive the focal length from a one-off calibration at that resolution (or store
the camera matrix and use it). Add the calibration images and the script to the repo.

### 14. Range from a single bounding-box dimension

**Current.** Range depends only on box width and an assumed real width. Truncation at
the frame edge, aspect-ratio changes and partial occlusion all bias the estimate, and
the EMA (`alpha = 0.2`) adds lag on top.

**Recommended.** Use both box dimensions with the known aspect ratio and reject boxes
touching the frame border. Keep the EMA but log the raw and smoothed values so the lag
is visible.

## Medium

### 15. A new telemetry stream every iteration

**Current.** `velocity_ned()` opens a stream, reads one sample, and drops the
generator. That costs a subscription round-trip per frame and can return a stale
sample, while adding latency inside the control loop.

**Recommended.** Run one persistent telemetry task that keeps the latest velocity,
attitude and battery state in a struct, and read from it in the loop. This also gives
you data to log.

### 16. Takeoff sequence

**Current.** Waits for `is_global_position_ok`, sets takeoff altitude, arms, takes off,
sleeps 10 s. It does not check `is_armable`, does not verify the airframe reached the
target altitude, and does not react if the mode changes during the wait.

**Recommended.** Poll `telemetry.armed()` and `telemetry.position().relative_altitude_m`
with a deadline, abort if the climb stalls, and abort if the flight mode is no longer
the expected one.

### 17. Three zero-filled observation dimensions

**Current.** Indices 3-5 are `torch.zeros(3) * 0.1`. Something was meant to be there
in training (previous action, target velocity, attitude?). The exported actor accepts
them without complaint, so a wrong guess produces plausible output rather than an error.

**Recommended.** Find the training environment's observation function and reproduce it
exactly, including scale factors. If those dimensions are genuinely unused, retrain
without them so the input has no dead channels.

### 18. Vertical control

**Current.** `action[2]` is discarded and `z` is always 0, so PX4's altitude hold does
the vertical work. Combined with `SAFE_ALTITUDE = 3.0` and no terrain awareness, the
aircraft will fly into anything at 3 m that the camera does not classify as close.

**Recommended.** Document this as a design choice (it is a reasonable one), and if
vertical control is ever needed, add the altitude error to the observation and scale
`action[2]` explicitly.

### 19. GUI lifecycle

**Current.** Only `q` ends the loop. Closing the window with the title-bar button leaves
`waitKey` running and the loop flying.

**Recommended.** Check `cv2.getWindowProperty(..., cv2.WND_PROP_VISIBLE)` in the display
thread, and set `exit_flag` when it drops. Then the loop's normal landing path runs.

### 20. No logging

**Current.** Console prints only: device, takeoff, offboard errors, landing.

**Recommended.** Per-frame CSV or JSONL with timestamp, loop period, raw and smoothed
range, box, track id, avoidance state, observation vector, action, setpoint and
telemetry. This is the single highest-value addition to the project: it makes every
claim in the README checkable and enables offline replay.

## What the code already does well

Worth keeping during any cleanup:

- The GUI runs on its own thread with a bounded queue, so `imshow` cannot block control.
- The avoidance majority filter is a sensible guard against single-frame depth noise,
  and its state is explicit rather than hidden in control flow.
- Commands are clipped to configured limits before they reach the offboard setpoint.
- Configuration is centralised, and the refactor preserved the ability to override the
  weight paths without editing code.
- The exported policy is small and self-contained (normaliser included), which makes
  inference deterministic and easy to test.
