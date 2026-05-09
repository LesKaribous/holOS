# Vision-assisted approach & grab — design study

Two related features for the Teensy-41 strategy that build on the existing
table-overhead vision pipeline:

1. **Approach-point pose confirmation**
   On arrival at an approach point (just before grabbing), ask holOS to
   confirm the robot's actual position via the overhead camera and recale
   the OTOS if it has drifted.

2. **On-board grab correction**
   At the same moment, capture a frame from a robot-mounted camera, send it
   to holOS for ArUco detection on the target object, and shift the grab
   point so the gripper actually meets the object instead of where OTOS
   *thinks* it should be.

These are two independent improvements in the same `collectStock` /
`storeStock` window. Feature 1 cancels global drift. Feature 2 cancels local
mis-registration of the object itself.

---

## What's already there

### Firmware ([firmware/teensy41/src/](../firmware/teensy41/src/))

- [`Localisation`](../firmware/teensy41/src/services/localisation/localisation.h) already exposes a complete vision-pose API. The hard work is done:
  - `requestHomographyCapture()` — frozen at start of `recalage()`
  - `requestVisionCalibration(known_pos)` — locks the OWN tag id + heading offset
  - `queryVisionPose(out_pose, timeout_ms)` — blocking pull of the latest own-tag pose, robot-frame heading already applied
  - `syncToVision(timeout_ms)` — convenience: query then `setPosition(vp)` overwrite
- [`JetsonBridge`](../firmware/teensy41/src/services/jetson/jetson_bridge.cpp) parses the reply frames `vis_pose(...)`, `vis_cal_done(...)`, `vis_h_locked(...)`. **Nothing for object detection yet.**
- [`strategy.cpp`](../firmware/teensy41/src/program/strategy.cpp) has two helpers `collectStock(target, tc, rc)` and `storeStock(target, tc, rc)` with the canonical sequence
  ```cpp
  motion.goAlign(approach, …)   // arrive at the approach point
  motion.collide(true)
  motion.goAlign(grab,     …)   // proceed to the grab pose
  motion.collide(false)
  actuators.moveElevator(rc, ElevatorPose::DOWN)
  …
  ```
  This is the spot to insert both features. APPROACH_OFFSET is `350 mm` for collect, `450 mm` for store. GRAB_OFFSET is `180 mm` / `50 mm`.
- No camera I/O on the Teensy itself — it has no USB host and no MIPI-CSI input. **Any embedded camera lives on the Jetson side.**

### Python / holOS ([software/](../software/))

- [`run.py`](../software/run.py) has the full T:vis subcommand router (`homography_capture`, `cal_request`, `pose_request`). New subcommands plug in there.
- [`_get_latest_own_pose()`](../software/run.py) already returns the current parallax-corrected pose with the captured heading offset applied. `pose_request` from firmware uses this.
- [`vision_pipelines_def.py`](../software/vision_pipelines_def.py) builds **one** pipeline (`localization`) wired to `source.video` reading the overhead camera (currently `http://127.0.0.1:5174/stream.mjpg`). Each pipeline is independent — adding a second one for the on-board camera is a one-`build_…()` job.
- The pose-comparison and calibration infrastructure on `/vision_debug` is already wired for the overhead camera.

### Conventions worth pinning

- `RobotCompass::AB / BC / CA` selects which actuator group is acting. Each maps to a face of the holonomic chassis, ~120° apart. The cam, if mounted, would belong to ONE face — most likely the same face as the active actuator group (so we always look in the direction we're about to grab).
- Strategy reads `ihm.isColor(Settings::BLUE)` before each block to pick blue-side or yellow-side POIs. The camera-mirror logic (already wired in `_apply_team`) handles the overhead camera.
- All coordinates in strategy code are in user-world frame: X+ left, Y+ up, origin = bottom-right corner of the table.
- `motion.goAlign(target, rc, theta)` orients the robot so face `rc` points along world heading `theta`. After arriving, the front of the actuator group `rc` faces the target.

---

## Feature 1 — Approach-point pose confirmation

### Goal

When the robot arrives at the `approach` waypoint (still ~350 mm away from
the object), pause briefly, ask the overhead camera "where am I really?",
and apply the correction before the slow / collision-sensitive `grab`
motion begins. OTOS drift over a 60-second match is the bug we're fixing.

### One-line in strategy

The intended call site in `collectStock`:

```cpp
async motion.goAlign(approach, rc, getCompassOrientation(tc));
visionConfirmPosition();   // ← new line, ~200-500 ms
RuntimeConfig::setInt("motion.timeout_ms", 2000);
motion.collide(true);
async motion.goAlign(grab, rc, getCompassOrientation(tc));
```

Same in `storeStock`, right after the approach. **Do not** call it after
`motion.collide(true) → goAlign(grab, …)` — by the time the gripper is
near the object, the camera FOV may be partially occluded by the robot
itself or its grippers.

### Implementation sketch

A thin strategy-local helper that wraps the existing `Localisation` API
with a sanity guard. Lives in `strategy.cpp` next to `waitMs()`:

```cpp
// Pull the latest overhead-vision pose and overwrite OTOS — but only
// if the delta is plausible (< MAX_TRUSTED_OFFSET_MM). Larger deltas
// are almost always a vision misdetection (wrong tag, parallax in a
// regime we haven't tuned for) and applying them would corrupt OTOS
// for the rest of the match. Returns true if we actually recaled.
constexpr float VISION_MAX_TRUSTED_OFFSET_MM = 100.0f;

static bool visionConfirmPosition(unsigned long timeout_ms = 400) {
    if (!localisation.isVisionCalibrated()) {
        return false;   // recalage hasn't run, no own tag locked
    }
    Vec3 vision_pose;
    if (!localisation.queryVisionPose(vision_pose, timeout_ms)) {
        return false;   // timeout / no fresh frame
    }
    Vec3 before = localisation.getPosition();
    Vec2 dxy(vision_pose.x - before.x, vision_pose.y - before.y);
    float dist = dxy.mag();
    if (dist > VISION_MAX_TRUSTED_OFFSET_MM) {
        Console::warn("Strategy")
            << "[vision-confirm] |Δ|=" << dist << "mm > "
            << VISION_MAX_TRUSTED_OFFSET_MM
            << "mm — REJECTED (likely misdetection)" << Console::endl;
        return false;
    }
    localisation.setPosition(vision_pose);
    Console::info("Strategy")
        << "[vision-confirm] |Δ|=" << dist << "mm  OTOS recaled"
        << Console::endl;
    return true;
}
```

Notes on the design:

- **Timeout 400 ms.** The pipeline ticks at 4 Hz, so worst case we wait
  one full tick + serial round-trip. The strategy is willing to spend
  this much before a critical grab.
- **MAX_TRUSTED_OFFSET_MM = 100 mm** is a defensive guard. After a clean
  recalage the residual is well under this. A single bad detection
  (tag occluded, opponent classified as own) typically jumps by hundreds
  of mm — easily filtered.
- **No motor-side action.** We rely on the next `motion.goAlign(grab, …)`
  to consume the recaled OTOS. No need to cancel / replan the in-flight
  motion since this runs *between* two blocking moves.
- **Heading is overwritten too.** `setPosition(Vec3)` writes x/y/theta.
  The vision heading is the OTOS-frame heading because the heading-offset
  capture (cal_request handshake) already maps tag→robot. If feature 1
  ever runs *before* a successful cal_request, `isVisionCalibrated()`
  returns false and we bail.
- **Order vs `motion.collide(true)`.** Set collide AFTER the confirm. The
  confirm doesn't move the robot, so collide state is irrelevant during
  it. Putting it before keeps the grab-collide window tight.

### Optional refinements (not v1)

- **Multi-sample vote.** If the vision is noisy, sample 2–3 frames and
  median-filter before applying. Costs ~150 ms extra per call.
- **Heading-only correction mode.** Sometimes only the heading has
  drifted (rotational play at the steppers). A `setHeadingOnly()` would
  recale `theta` but leave xy. Useful when the xy delta is high (untrust)
  but the heading delta is small.
- **Auto-disable after the second-to-last block.** If we're tight on
  time we could skip the confirm in the last 10 s.

### Observability

The vision-debug log already shows recalage results. For runtime confirms
we'd want a per-block trace. Either:

- Reuse the existing `_vlog` channel (firmware doesn't write to it
  directly — the log lives in `run.py`). On the Python side, the
  `_on_vis pose_request` handler already runs every confirm; just remove
  the "don't log pose_request" guard and add a 1 Hz throttled log line
  with the requested pose vs delivered pose.
- Or have the firmware emit a `Console::info("vision-confirm")` line
  that the operator reads from the Teensy serial.

---

## Feature 2 — On-board camera grab correction

### Goal

Just before the final `grab` motion, snap a frame from a camera mounted
on the robot, look for an ArUco tag on the target object, compute the
object's position in the robot frame, and shift the planned grab target
by the residual.

This compensates for:

- Object placed by humans at slightly the wrong spot (Coupe rules don't
  give us mm-perfect placement).
- Object that has been bumped by an opponent.
- Cumulative offsets the overhead camera can't see (occlusion by the
  robot's own body when standing right over the object).

### What needs to exist that doesn't yet

| Layer | What's missing |
|---|---|
| **Hardware** | A camera on the robot. Probably a USB cam on the Jetson, or a CSI cam if Jetson is the Orin Nano. |
| **Cabling** | Onboard cam plugs into the Jetson, NOT the Teensy. Jetson exposes the stream over a local URL the way the overhead one does. |
| **Mechanical** | Cam mounted on a face that aims toward the gripper / table. Fixed orientation (or a known per-`RobotCompass` mapping). |
| **Calibration** | A `homography` from camera image → robot-frame ground plane, captured once with known reference tags. |
| **Pipeline** | A second pipeline `onboard` running an ArUco detector over the onboard stream. Pull-based (only ticks when asked) to save Jetson CPU. |
| **Bridge** | A new `T:vis grab_request` ↔ `vis_grab_done` round-trip. |
| **Firmware** | A new `Localisation::requestObjectGrabHint(rc)` + reply handler + a strategy helper that consumes the hint. |

### Coordinate frames involved

```
   world frame           robot frame            camera frame          image frame
   ─────────────         ─────────────          ──────────────        ──────────────
   x_world, y_world  →   x_robot, y_robot   →  x_cam, y_cam, z_cam → u, v (pixels)
   (table mm)            (origin = robot      (origin = camera      (raw cv2)
                          center, X = front     pinhole, optical
                          of face `rc`)         axis = +Z)
```

For the on-board cam:

- **Camera mount → robot frame** is a static transform `T_cam_in_robot`.
  Position (`cx, cy, cz`) and orientation (`yaw, pitch, roll`) are
  measured/calibrated once.
- **Object → camera frame** comes from ArUco's `solvePnP` if we have
  intrinsics + tag size. Or we can short-circuit with a homography
  fitted from a known-pose calibration board (much simpler).
- **Object → robot frame** = `T_cam_in_robot * (x_cam, y_cam)`.
- **Robot → world** is the live `motion.estimatedPosition()`.

For the simplest v1 we can drop the 3D and assume:
- Cam looks straight down at the table from a fixed height above the gripper
- Image plane is parallel to the ground plane → just an `image-mm` homography
- Output: `(dx_mm, dy_mm)` of the object center relative to a fixed
  reference point in the image (e.g. the image center, or a chosen
  pixel that maps to "where the gripper expects the object to be")

### Pipeline graph (Python)

A second `build_onboard()` builder added to `vision_pipelines_def.py`,
not auto-enabled:

```
source.video (onboard-stream URL)
   ↓
preprocess (passthrough or grayscale)
   ↓
aruco (4x4_50, no team filter — looks for object IDs)
   ↓
output.aruco_list (feed_id "onboard_aruco")
   ↓
output (preview, feed_id "onboard_preview", low fps)
```

The pipeline ticks at 1–2 Hz when nobody is asking. When a `grab_request`
comes in, the existing pipeline tick is fine — we just snap the latest
result. (No need for a true on-demand single-shot mode for v1; rate-
limiting + last-frame cache covers it.)

### Firmware → holOS protocol

Same `T:vis` channel as the rest. New subcommand:

```
firmware → holOS:  "T:vis grab_request rc=AB tag=42 expected_x=1100 expected_y=1825"
                    │            │       │     │       │              └─ where strategy
                    │            │       │     │       │                 thought the object
                    │            │       │     │       └─ optional: object id we want.
                    │            │       │     │          If absent, holOS picks the
                    │            │       │     │          closest tag in the FOV.
                    │            │       │     └─ which face is looking
                    │            │       └─ subcommand
                    │            └─ vis channel
                    └─ T: telemetry frame

holOS → firmware:  "vis_grab_done(found=1,tag=42,dx=12.3,dy=-4.5,confidence=0.91)"
                  or "vis_grab_done(found=0,reason=no_tag_in_fov)"
```

`dx, dy` are the **correction** to apply to the strategy's expected grab
target, in the **world frame** (already rotated by the robot's heading).
This keeps the strategy code dumb: it just adds dx/dy to its target
vector. Heavy math (camera→robot→world) lives in the Python side where
homography + intrinsics are loaded.

The firmware-side reply parser mirrors the existing `vis_cal_done` /
`vis_pose` handlers. One new method on `Localisation`:

```cpp
// Returns true if a usable hint came back. dx_mm/dy_mm filled with the
// world-frame correction; tag_id with the matched ArUco id (0 = no
// match, "blind" grab).
bool requestObjectGrabHint(RobotCompass rc,
                           int expected_tag_id,
                           Vec2 expected_pos,
                           Vec2& out_correction_mm,
                           int& out_tag_id,
                           float& out_confidence,
                           unsigned long timeout_ms = 600);
```

Spin-wait pattern identical to `queryVisionPose`. Daemon thread on
holOS does the heavy lifting and sends the reply.

### Strategy-level helper

```cpp
// Adjusts `grab` and `target` in-place when vision finds the object.
// On failure (no tag seen, low confidence, large correction) leaves
// them untouched — the strategy falls back to the open-loop grab.
constexpr float GRAB_HINT_MAX_MM = 80.0f;
constexpr float GRAB_HINT_MIN_CONFIDENCE = 0.5f;

static bool refineGrabPointWithCamera(Vec2& target, Vec2& grab,
                                      RobotCompass rc,
                                      int expected_tag_id) {
    Vec2 correction;
    int  found_tag = 0;
    float confidence = 0.0f;
    bool ok = localisation.requestObjectGrabHint(
        rc, expected_tag_id, target,
        correction, found_tag, confidence);
    if (!ok) {
        Console::warn("Strategy") << "[grab-refine] no hint, blind grab"
                                  << Console::endl;
        return false;
    }
    if (confidence < GRAB_HINT_MIN_CONFIDENCE) return false;
    if (correction.mag() > GRAB_HINT_MAX_MM) {
        Console::warn("Strategy")
            << "[grab-refine] correction " << correction.mag()
            << "mm > clamp — REJECTED" << Console::endl;
        return false;
    }
    target += correction;
    grab   += correction;
    Console::info("Strategy")
        << "[grab-refine] tag #" << found_tag
        << " Δ=(" << correction.x << "," << correction.y
        << ") c=" << confidence << Console::endl;
    return true;
}
```

Call site in `collectStock`:

```cpp
async motion.goAlign(approach, rc, getCompassOrientation(tc));
visionConfirmPosition();          // feature 1
refineGrabPointWithCamera(target, grab, rc, /* tag_id */ -1);   // feature 2
RuntimeConfig::setInt("motion.timeout_ms", 2000);
motion.collide(true);
async motion.goAlign(grab, rc, getCompassOrientation(tc));
```

`-1` for `expected_tag_id` means "look for any tag, pick the closest to
the expected position". Once the per-zone object IDs are known, replace
with the literal id (e.g. 30 for stockBlue_01) so the system rejects
stray markers.

### holOS-side handler

In [run.py](../software/run.py) `_on_vis`, branch the new subcommand:

```python
elif line.startswith('grab_request'):
    rc       = _kv(line, 'rc',         str,   'AB')
    expected = (_kv(line, 'expected_x', float, 0.0),
                _kv(line, 'expected_y', float, 0.0))
    tag_id   = _kv(line, 'tag',         int,   -1)
    _vlog(f'rx ← T:vis grab_request rc={rc} tag={tag_id} '
          f'expected=({expected[0]:.0f},{expected[1]:.0f})')
    def _do_grab():
        result = _onboard_pick_grab_target(rc, tag_id, expected)
        if result is None:
            _send('vis_grab_done(found=0,reason=no_tag_in_fov)')
            return
        _send(f"vis_grab_done(found=1,tag={result['tag_id']},"
              f"dx={result['dx_mm']:.1f},dy={result['dy_mm']:.1f},"
              f"confidence={result['confidence']:.2f})")
    threading.Thread(target=_do_grab, daemon=True,
                     name='vis-grab').start()
```

`_onboard_pick_grab_target(rc, tag_id, expected)` would:

1. Resolve the onboard pipeline's aruco list from its node outputs.
2. Filter to the expected tag id (or all object-range tags).
3. For each candidate: pixel → camera-frame mm via the cam homography
   → robot frame via `T_cam_in_robot[rc]` → world frame via the live
   robot pose (read from `_hw_tel_data['pos']`).
4. Pick the candidate closest to `expected`. Compute correction =
   tag_world − expected.
5. Return tag id, dx/dy, confidence (a function of #corners detected,
   reprojection residual, marker visible-area).

Failure modes that should each return None with a reason:

- No frame from the onboard pipeline (camera not streaming).
- Pipeline disabled.
- No tags detected this tick.
- Best candidate too far from `expected` (probably wrong object).
- Live robot pose unknown (no telemetry).

### Calibration of the on-board camera

Two things to capture, both **once** per physical mount:

1. **`T_cam_in_robot[rc]`** — the static transform from robot center
   (origin) to the camera. Either measured by hand (cx, cy, cz, yaw,
   pitch, roll) or via a one-shot procedure: place a known calibration
   tag at a known position relative to the robot, capture, solve.
2. **Image → ground homography** — capture the calibration tag at known
   ground positions across the FOV (4+ poses). Run findHomography. Save
   to `software/vision/calibrations/onboard_homography.json`.

A small CLI / web UI under `/vision_debug` would help here, but for v1
a manual measurement + JSON edit is fine.

### Mounting expectations (to confirm with you)

These I'd want to confirm before writing firmware:

- **One camera or several?** If one per face (AB / BC / CA), each `rc`
  needs its own mount transform + homography. If one camera mounted
  fixed, the strategy must rotate the robot first so the active face is
  pointing toward the object — which is what `goAlign(approach, rc,
  …)` already does, so it's fine.
- **FOV vs grab distance.** With APPROACH_OFFSET = 350 mm, the object
  needs to be visible from 350 mm away with enough resolution. A 1080p
  camera at ~300 mm distance gives ~1 mm/pixel — workable.
- **Lighting.** Match arenas are evenly lit but the robot's own body
  casts a shadow on the immediate gripper area. Cam should be aimed at
  the object FROM ABOVE the gripper, not BEHIND it.

---

## Test plan (when we get to it)

### Feature 1

1. Lay out a fake match: place an obstacle near the approach point so
   the robot has to nudge its OTOS during transit.
2. Tape-measure the actual robot position at the approach point.
3. Compare `localisation.getPosition()` before vs. after
   `visionConfirmPosition()`.
4. Confirm the next `motion.goAlign(grab, …)` lands within ±10 mm of
   the tape-measured grab target.
5. Sabotage: artificially set OTOS off by 200 mm before the approach.
   Vision should reject (>100 mm clamp) and log a warning.

### Feature 2

1. Place a tagged object exactly at the expected position. Confirm
   `correction = (0, 0) ± noise`.
2. Shift the object by 30 mm. Confirm `correction ≈ (30, 0)`.
3. Shift by 200 mm. Confirm rejection (>80 mm clamp).
4. Cover the tag. Confirm `vis_grab_done(found=0)` returns within
   timeout and strategy falls through to blind grab.
5. Stress: the opponent passes between the cam and the object.
   Confirm the temporary occlusion doesn't poison the result (we read
   the latest detection, not a stale one).

---

## Order of work (proposed)

1. **Feature 1 first** — small, no new hardware, immediately useful.
   ~half a day of work + tuning.
2. **Onboard cam wiring** — pick a camera, mount it, route USB to
   Jetson, expose stream URL. Hardware-side, mostly outside this repo.
3. **Onboard pipeline + homography calibration** — Python side. New
   `build_onboard()` + a one-off calibration script.
4. **Feature 2 firmware-side** — bridge subcommand + Localisation API
   + strategy helper + integrate in `collectStock` / `storeStock`.
5. **Field-test loop** — match with both features active, log
   refusal/acceptance counts, tune the clamps.

---

## Open questions for you

- **Object ArUco IDs** — what range will you use for game elements?
  We've reserved 1–10 for robots, 20–23 for table anchors. Suggest
  30+ for objects to leave room for both.
- **Onboard cam location** — one per face or one fixed? Resolution?
  Approximate height above the table?
- **Failure policy** — when the vision returns no fix at the approach
  point, do you want to (a) abort the block and try another, or
  (b) fall through to the open-loop grab? Current sketch is (b).
- **Heading correction at confirm** — overwrite the OTOS heading too,
  or only xy? Overwriting heading needs the cal_request handshake to
  have run reliably, otherwise we might inject 180° errors.
