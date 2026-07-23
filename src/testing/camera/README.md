# Camera / Vision test suite

Everything the camera is responsible for, split into **one test per job** so a
failure points at exactly one subsystem. Work top-to-bottom: don't trust the
pillar test until Job 1 (the pixels themselves) is green, because every
detector downstream is only as correct as the frame it's handed.

| # | Job | Source file | Test file | Output it produces |
|---|-----|-------------|-----------|--------------------|
| 1 | Frame acquisition | `perception/camera.py` | `frame_source_test.py` | RGB array + JPEG stream |
| 2 | Pillar detect + avoid | `perception/pillar_detection.py` | `pillar_test.py` | colour, pass-side, steer, distance |
| 3 | Corner markers → laps | `perception/corner_detection.py` | `corner_test.py` | orange/blue edges |
| 4 | Parking zone | `perception/finish_detection.py` | `parking_test.py` | magenta confirmed |
| 5 | Lane / wall centering | `perception/track_detection.py` | `lane_test.py` | corridor offset |
| 6 | Integration / gating | `runtime.py` | `integration_test.py` | laps, mode, state gating |

Plus two calibration tools: `calibrate_hsv.py` (colour bounds) and
`calibrate_focal.py` (pillar-distance focal length).

---

## Each test has two modes

* **`--selfcheck`** — synthetic frames, **no camera needed**. Verifies the
  *logic* (thresholds, sign conventions, hysteresis, lap math). Run these
  anywhere, including CI. They must ALL pass before you touch hardware.
* **live** (no flag) — grabs real pixels through the project `Camera` object
  and prints telemetry every tick. This is the **in-person** test: you move a
  pillar / marker / wall in front of the lens and watch the numbers. Ctrl-C to
  stop. Annotated debug frames are dropped in `testing/camera/_debug_out/`
  (git-ignored) for you to `scp` back and eyeball.

Job 6 has no live mode — it's pure logic; its hardware half is the
"On-robot end-to-end" section near the bottom.

## Setup

Run everything from the **`src/` directory**:

```bash
cd src
```

On the Pi you need the real-camera stack (the rest is stdlib):

```bash
sudo apt install -y python3-picamera2      # Pi Camera on Bookworm
pip install numpy pillow opencv-python     # perception maths + encode/masks
```

`--selfcheck` and every live test need `numpy` + `opencv-python`. `picamera2`
is only needed for the actual Pi Camera; without it the suite falls back to a
USB webcam (OpenCV) and then to synthetic frames.

## Fast path: prove the logic first (no hardware)

```bash
cd src
for j in frame_source pillar corner parking lane; do
  echo "== $j =="; python -m testing.camera.${j}_test --selfcheck || echo "  ^ FAILED"
done
python -m testing.camera.integration_test
```

Every block ends in `SELF-CHECK: PASS` / `INTEGRATION: PASS`. If any line reads
`FAIL:` you have a code/config regression — fix that before the mat session,
because live testing can't distinguish "bad code" from "bad lighting".

---

## Job 1 — Frame acquisition

```bash
python -m testing.camera.frame_source_test          # live; point at a PURE RED object
python -m testing.camera.frame_source_test --selfcheck
```

**Do in person:** fill the centre of the frame with a known **pure red** object.

**Expected live output:**
```
source: picamera2                 <- NOT synthetic, NOT opencv, on the Pi
shape:  (240, 320, 3)   (expected (240, 320, 3))
fps:    ~12-15
centre patch  R=~200  G=low  B=low     <- R dominant == channel order OK
JPEG: 4000-9000 bytes, magic OK
saved .../job1_frame.png
```
Open `job1_frame.png`: **red must look red, not blue; no pink cast anywhere.**

**Known bugs & fixes**

| Symptom | Cause | Fix |
|---|---|---|
| `source: synthetic` on the Pi | picamera2 & OpenCV both failed to open | `libcamera-hello` to test the sensor; reseat the ribbon; `sudo apt install python3-picamera2`; check `Camera._open_real` logs at DEBUG |
| Red object shows **B** dominant / image is pink/magenta | R/B channels swapped | The swap lives in `Camera._swap_rb` / `get_frame` / `_real_jpeg` (`camera.py`). Picamera2 `RGB888` actually returns BGR, so `get_frame` must flip once. Don't flip twice — flipping in both `get_frame` and `_real_jpeg` cancels for the stream but leaves perception wrong (or vice-versa). Test the array (this test) AND the dashboard image together. |
| `shape` is `(240,320,4)` | camera returns RGBA/XRGB | `_swap_rb` already handles 4-channel; make sure perception still gets 3 — check `create_preview_configuration` format |
| `fps` far below 15 | resolution/format too heavy, or CPU pegged | lower `FRAME_WIDTH/HEIGHT` in `camera_config.py`, or the fps arg to `Camera()` |
| `get_frame() returned None` | real backend read failed this tick | transient is fine; constant means the device died — re-open |

---

## Job 2 — Pillar detection & avoidance

```bash
python -m testing.camera.pillar_test          # live; real red/green pillar
python -m testing.camera.pillar_test --selfcheck
```

**Do in person:**
1. Hold a **red** pillar centre-frame → expect `RED … pass RIGHT steer=+NN`.
2. Hold a **green** pillar → expect `GREEN … pass LEFT steer=-NN`.
3. **Slide** it left→right across the frame: sign stays correct, `|steer|`
   never drops below **18°**, never exceeds **60°**.
4. **Walk** it toward the camera: `~mm` shrinks. (Its absolute value is only
   right after `calibrate_focal.py` — Job 2 checks the *trend*, not the value.)
5. Cover the pillar: after **5** empty frames you get `CLEARED`.

**Expected live output:**
```
RED    cx=178 area=  920 -> pass RIGHT steer=+27 ~430mm  new=1 clr=0
RED    cx=181 area= 1100 -> pass RIGHT steer=+26 ~390mm  new=0 clr=0
--     (no pillar)
CLEARED (RED)
```

**Known bugs & fixes**

| Symptom | Cause | Fix |
|---|---|---|
| Red/green pillar never detected | HSV bounds off for your lighting | `python -m testing.camera.calibrate_hsv --color RED` (and `GREEN`), paste bounds into `camera_config.py` |
| Red detected only sometimes | red wraps the 0/180 hue boundary; only one of the two ranges is tuned | keep BOTH `HSV_RED_LOW1/HIGH1` and `HSV_RED_LOW2/HIGH2`; widen the one your lighting hits |
| Detects tiny specks as pillars | `MIN_PILLAR_AREA` too low for the noise | raise it in `camera_config.py`; confirm with the `--selfcheck` area test |
| Steer sign backwards (turns INTO the pillar) | `decide_direction` / `compute_steer_angle` sign, OR the servo wiring is mirrored | this test proves the *software* sign; if software is right but the car still turns wrong, it's `STEER_CENTER_DEGREE`/servo polarity in firmware, not here |
| `~mm` wildly wrong | `CAMERA_FOCAL_PX` uncalibrated | run `calibrate_focal.py` |
| Avoidance ends too early / never ends | clear logic: shrinking + within `PILLAR_CLEAR_DISTANCE_MM`, or 5 lost frames | tune `PILLAR_CLEAR_DISTANCE_MM` / `PILLAR_LOST_FRAMES`; the two clear paths are both in the `--selfcheck` |

---

## Job 3 — Corner markers → laps

```bash
python -m testing.camera.corner_test          # live; orange/blue floor line
python -m testing.camera.corner_test --selfcheck
```

**Do in person:** slide an orange (or blue) floor strip through the **bottom**
of the frame. Expect **exactly one** `new=1` as it enters and **one** `CLEARED`
a few frames after it leaves — not a burst.

> The camera only reports the *edges*. Turning edges into a lap count is Job 6.

**Expected live output:**
```
ORANGE new=1 clr=0
ORANGE new=0 clr=0
--
CLEARED (ORANGE)
```

**Known bugs & fixes**

| Symptom | Cause | Fix |
|---|---|---|
| Orange reads as red, or blue as something else | orange/blue bounds overlap pillar/parking hues | bounds are deliberately spaced (`camera_config.py:24-31`); re-tune with `calibrate_hsv --color ORANGE`/`BLUE`; keep them in the documented gaps |
| Marker flickers new/clear rapidly | thin 20 mm line dips below `MIN_CORNER_MARKER_AREA` | that's why `MIN_CORNER_MARKER_AREA` (150) is lower than a pillar; lower it more or raise the ROI light; `CORNER_MARKER_LOST_FRAMES` hysteresis absorbs single drops |
| Corner miscounts a lap | counting is Job 6, not here | if edges here are clean but laps are wrong, the bug is in `runtime.py` — run `integration_test.py` |

---

## Job 4 — Parking zone (magenta)

```bash
python -m testing.camera.parking_test          # live; magenta parking line
python -m testing.camera.parking_test --selfcheck
```

**Do in person:** show the magenta line steadily → after **5** consecutive
frames you get `CONFIRMED` **once**, then it latches (silent forever after).
Flash it for <5 frames → **no** confirm.

**Expected live output:**
```
seen cx=160 area= 640  consec->
seen cx=161 area= 655  consec->
CONFIRMED cx=160 area=655  (latched)
--
```

**Known bugs & fixes**

| Symptom | Cause | Fix |
|---|---|---|
| Magenta never confirms | bounds off, or marker too small | `calibrate_hsv --color MAGENTA`; check `MIN_PARKING_MARKER_AREA` |
| Confirms off a brief glint | `PARKING_CONFIRM_FRAMES` too low | raise it in `camera_config.py` |
| Re-fires repeatedly | latch broke (`_raised` reset) | should latch until `reset()` at run start — see `ParkingZoneTracker`; the `--selfcheck` guards this |
| Red pillar sometimes reads as magenta | red-high range and magenta range are adjacent on the hue circle | keep `HSV_RED_HIGH2` ending below `HSV_MAGENTA_LOW`; re-tune both |

---

## Job 5 — Lane / wall centering

```bash
python -m testing.camera.lane_test          # live; robot in a real corridor
python -m testing.camera.lane_test --selfcheck
```

**Do in person:** with the robot between two black walls, physically slide it
toward the **left** wall. `off` should go **positive** and `nudge` **positive**
(steer right, back to centre). Slide right → both negative. Remove one wall →
`conf=0.5`. Point at open white mat → `INVALID`.

**Expected live output:**
```
valid off=+34 conf=1.0  L=118 R=286  nudge=+2.0
valid off=-40 conf=0.5  L=None R=232 nudge=-2.4
INVALID (no walls)
```

**Known bugs & fixes**

| Symptom | Cause | Fix |
|---|---|---|
| White mat detected as wall | `LANE_BLACK_S_MAX`/`V_MAX` too high | `calibrate_hsv --color BLACK` gives suggested S/V max; lower them in `thresholds.py` |
| Walls never detected | S/V max too low, or ROI misses the wall base | raise the maxes; check `LANE_ROI_TOP_FRAC`/`BOTTOM_FRAC` frame the wall foot |
| Offset sign backwards (steers into the wall) | sign convention | `+offset` = corridor centre right of image centre = drifted left = nudge right; verified in `--selfcheck` |
| Car hugs a wall on straights | `LANE_DEFAULT_HALF_WIDTH_PX` guess wrong for one-wall case, or `INNER_WALL_BIAS_PX` too strong (OPEN only) | retune both on the mat |
| Jittery centering | deadband too small / gain too high | raise `LANE_OFFSET_DEADBAND_PX`, lower `LANE_STEER_KP`, cap via `LANE_MAX_NUDGE_DEG` |

---

## Job 6 — Integration / state gating

```bash
python -m testing.camera.integration_test     # no hardware
```

Proves the glue in `runtime.py` around the detectors:

* a corner is counted **once**, on the completion edge — `CORNERS_PER_LAP`
  (4) completions = 1 lap; `TARGET_LAPS` (3) = `THREE_LAPS_COMPLETE` once;
* run direction latches from the **first** corner marker's colour;
* the **OPEN** verdict fires after a full lap with no pillar;
* `run_pillars` is off in an OPEN run;
* `_VISION_ACTIVE_STATES` is exactly the 4 driving states (checked against the
  real `runtime` tuple when its deps import; else against the contract).

Expect `INTEGRATION: PASS`.

### On-robot end-to-end (the hardware half of Job 6)

This can only be signed off on the robot, with the ESP32 connected:

1. Put the car in a corridor, one red pillar ahead. Start a run (`main.py
   --mode run`, or the dashboard).
2. Watch the logs / dashboard: `PILLAR_DETECTED_RED` → state `AVOID_OBSTACLE`
   → a steer command on the serial wire (confirm the servo actually moves the
   right way) → pass the pillar → `OBSTACLE_CLEARED` → back to `FOLLOW_TRACK`.
3. Drive a full lap: 4 corners → `Lap complete: 1/3`. Three laps →
   `THREE_LAPS_COMPLETE` → PARK/STOP.
4. Repeat once with a **green** pillar (passes on the other side) and once with
   an **OPEN** mat (no pillars) to confirm mode auto-detection.

If a step here misbehaves but every `--selfcheck` above is green, the bug is in
the wiring (`runtime.py`), the serial link, or the firmware — **not** in the
camera/perception code you just isolated.

---

## Calibration tools

Run these whenever you change venue/lighting — the config defaults explicitly
say they need an on-mat pass.

**Colour bounds** — repeat per colour, paste the suggested lines into
`config/camera_config.py` (or `thresholds.py` for `BLACK`):
```bash
python -m testing.camera.calibrate_hsv --color RED
python -m testing.camera.calibrate_hsv --color GREEN
python -m testing.camera.calibrate_hsv --color ORANGE
python -m testing.camera.calibrate_hsv --color BLUE
python -m testing.camera.calibrate_hsv --color MAGENTA
python -m testing.camera.calibrate_hsv --color BLACK      # walls -> S/V max
```
Fill the drawn centre box with one clean sample; it averages ~40 frames and
prints ready-to-paste bounds plus a preview so you can confirm the box sat on
the colour.

**Pillar distance focal length** — put a pillar at a tape-measured distance:
```bash
python -m testing.camera.calibrate_focal --distance-mm 1000
```
Paste the printed `CAMERA_FOCAL_PX` into `config/camera_config.py`, then re-run
`pillar_test` and confirm `~mm` matches your tape at 2–3 distances.

---

## Exhaustive sign-off checklist

A perfect camera = every box below ticked, in order. `[S]` = covered by
`--selfcheck` (logic), `[L]` = must be verified live/in-person, `[C]` =
calibration step.

### 0. Calibration (do first, on the competition mat)
- [ ] `[C]` HSV bounds calibrated for RED, GREEN, ORANGE, BLUE, MAGENTA under match lighting
- [ ] `[C]` BLACK wall S/V max calibrated (white mat NOT in the mask)
- [ ] `[C]` `CAMERA_FOCAL_PX` calibrated against a tape-measured pillar

### 1. Frame acquisition
- [ ] `[L]` Backend is `picamera2` on the Pi (not opencv/synthetic)
- [ ] `[S]` `[L]` Frame shape is `(240, 320, 3)`
- [ ] `[L]` Measured FPS ≈ 15
- [ ] `[L]` RGB order correct — pure-red object → R channel dominant, no pink cast
- [ ] `[S]` `[L]` `get_jpeg()` returns valid JPEG; dashboard MJPEG feed live
- [ ] `[S]` No-camera fallback degrades to synthetic without crashing

### 2. Pillar detection & avoidance
- [ ] `[S]` `[L]` Presence: pillar → blob; empty frame → none
- [ ] `[S]` Sub-`MIN_PILLAR_AREA` blob ignored; `[L]` tune the trigger distance
- [ ] `[L]` RED classified RED, GREEN classified GREEN under match lighting
- [ ] `[S]` Pass-side: RED→RIGHT, GREEN→LEFT
- [ ] `[S]` `[L]` Steer sign correct + `18° ≤ |steer| ≤ 60°` swept across frame
- [ ] `[S]` Distance trend correct; `[C]`/`[L]` absolute value matches tape
- [ ] `[S]` `new_detection` fires exactly once per appearance
- [ ] `[S]` `[L]` `cleared` fires via shrink+close AND via 5 lost frames
- [ ] `[S]` Single dropped frame does NOT clear (hysteresis)

### 3. Corner markers → laps
- [ ] `[S]` `[L]` Orange vs blue classified correctly
- [ ] `[S]` Sub-`MIN_CORNER_MARKER_AREA` noise ignored; thin line still triggers
- [ ] `[S]` `[L]` One `new_detection` on entry, one `cleared` on exit; 4-frame hysteresis
- [ ] `[S]` Lap math: 4 corners = 1 lap, 12 = 3 laps → `THREE_LAPS_COMPLETE` once
- [ ] `[S]` `[L]` Direction latches from the first marker (orange=CW / blue=CCW), sticky

### 4. Parking zone
- [ ] `[S]` `[L]` Magenta detected; `MIN_PARKING_MARKER_AREA` respected
- [ ] `[S]` `[L]` 5 consecutive → confirm once, then latched
- [ ] `[S]` `[L]` <5-frame flash does NOT confirm

### 5. Lane / wall centering
- [ ] `[S]` `[L]` White mat excluded from wall mask
- [ ] `[S]` Both walls → `confidence=1.0`, true midpoint
- [ ] `[S]` One wall → `confidence=0.5` (assumed half-width)
- [ ] `[S]` `[L]` No walls (white) → `valid=False`
- [ ] `[S]` `[L]` Offset sign: drift left → `+offset` → `+nudge` (steer right)
- [ ] `[S]` ROI limited to the lower band (0.45–0.95 of height)

### 6. Integration / state gating
- [ ] `[S]` Vision runs only in the 4 driving states (no stray event in `WAIT_FOR_START`)
- [ ] `[S]` OPEN run gates pillar detection off; lane centering keeps running
- [ ] `[L]` End-to-end: pillar → `AVOID_OBSTACLE` → steer on the wire → cleared → `FOLLOW_TRACK`
- [ ] `[L]` Full OBSTACLE run and full OPEN run both finish correctly on the robot

When every box is ticked, the camera is done.
