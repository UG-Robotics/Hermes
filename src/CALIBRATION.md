# Hermes On-Mat Calibration & Tuning Guide

Everything in this guide is a **constant in `src/config/`** — no logic changes
needed. Tune in roughly the order below; each section says which file to edit,
what the symptom of a bad value looks like, and how to find a good one. Bring a
tape measure, the real mat, and a charged battery (gains that work at 8.4 V
drift as the pack sags).

> The defaults shipped in the repo are deliberately **conservative** (slower,
> wider margins) so the first runs fail safe rather than into a wall. Expect to
> make the car faster/tighter once each subsystem is trusted.

---

## 0. Before you tune anything

- Install perception deps on the Pi: `pip install opencv-python numpy pillow`
  (without `opencv-python`, every detector logs a warning and no-ops — the car
  will drive blind).
- Flash `firmware/esp_controller/` to the ESP32, then confirm the servo
  centre/limits and IMU sign (Sections 1–2) **before** trusting any autonomy.
- Sanity-check the wiring convention: `hardware/tof.py` and `firmware/.../tof.cpp`
  both assume **tof1 = LEFT, tof2 = RIGHT**. If a hand near the left sensor
  moves the *right* number on the dashboard, swap the sensors or flip the
  mapping in one place.

---

## 1. Servo centre & steering limits  → `firmware/esp_controller/config.h`

| Constant | Meaning | How to set |
|---|---|---|
| `SERVO_CENTER` | duty that points the wheels straight | Command `STEER,0`; adjust until the car rolls dead straight over 2 m. |
| `SERVO_LEFT` / `SERVO_RIGHT` | full-lock duties | Command ±90; set so the wheels hit mechanical max **without buzzing/stalling** the servo. |

Mirror the same physical straight-ahead in `robot_config.py:STEER_CENTER_DEGREE`
if you change the mechanical trim.

## 2. IMU heading sign  → `firmware/esp_controller/config.h`

`IMU_GZ_SIGN` must make a **clockwise (rightward) spin produce an increasing
heading**. Spin the chassis right by hand and watch `heading_deg` on the
dashboard; if it *decreases*, set `IMU_GZ_SIGN = -1`. Everything downstream
(`+steer = right`) depends on this being correct.

## 3. Heading-hold PID  → `config/pid_config.py`

This is the loop that keeps the car straight and executes avoidance turns.

- `STEER_PID_KP` (1.6): raise until the car corrects drift briskly; back off if
  it visibly weaves (oscillates) down a straight.
- `STEER_PID_KD` (0.35): raise to damp that weave.
- `STEER_PID_KI` (0.02): keep small; only for steady-state pull. Too high →
  slow snaking.
- `STEER_SLEW_MAX_DEG_PER_TICK` (12): lower for smoother, slower servo motion;
  raise if avoidance turns feel sluggish.

**Test:** drive a straight at speed — it should track arrow-straight. Then a
gentle S — it should settle without hunting.

## 4. Camera & colour thresholds  → `config/camera_config.py`

Use `testing/vision/pillar_vision_test.py` on the Pi, under **competition
lighting**, to read HSV values off the actual pillars/markers.

| Constants | Symptom if wrong |
|---|---|
| `HSV_RED_*`, `HSV_GREEN_*` | pillars missed, or mat/lighting misclassified as a pillar |
| `HSV_ORANGE_*`, `HSV_BLUE_*` | corners miscounted → laps wrong, wrong direction latched |
| `HSV_MAGENTA_*` | parking lot never confirmed (obstacle run won't park) |
| `MIN_PILLAR_AREA`, `MIN_CORNER_MARKER_AREA`, `MIN_PARKING_MARKER_AREA` | too low = noise triggers detections; too high = real markers ignored |

The four corner/pillar hue bands are chosen **not to overlap** — keep that gap
if you retune (see the comment block in the file).

## 5. Monocular distance / focal length  → `config/camera_config.py`

`CAMERA_FOCAL_PX` (350) sets the pillar distance estimate
`distance ≈ real_width × focal / pixel_width`. Calibrate: place a pillar at a
known distance (say 1000 mm), read its pixel width from the vision tester, solve
`focal = distance × pixel_width / PILLAR_REAL_DIAMETER_MM`. Then tune
`PILLAR_CLEAR_DISTANCE_MM` (180) so `OBSTACLE_CLEARED` fires just as you draw
alongside the pillar — too early aborts the pass, too late clips it.

## 6. Lane / wall detection  → `config/thresholds.py`

- `LANE_BLACK_S_MAX` / `LANE_BLACK_V_MAX`: the "this pixel is a black wall"
  cutoff. If shadows read as wall, lower `V_MAX`; if a dim wall is missed,
  raise it.
- `LANE_ROI_TOP_FRAC` / `BOTTOM_FRAC` (0.45–0.95): the band of frame searched
  for walls. Raise the top if the car's own bumper/horizon leaks in.
- `LANE_DEFAULT_HALF_WIDTH_PX` (110): where the corridor centre is assumed to
  be when only one wall is visible. Set it to roughly half the corridor's pixel
  width at mid-ROI for your camera height.
- `LANE_STEER_KP` (0.06) / `LANE_MAX_NUDGE_DEG` (4): how hard vision drags the
  heading target toward corridor centre. Raise KP if centering is lazy; lower
  if the car wanders toward walls.

## 7. ToF wall safety & speed  → `config/thresholds.py`

- `TOF_WALL_WARNING_MM` (150) / `TOF_WALL_CRITICAL_MM` (80): distances at which
  the car softens avoidance and slows. Widen if it clips walls, tighten if it's
  needlessly timid in the 1000 mm obstacle corridor.
- `SPEED_SCALE_WALL_WARNING/CRITICAL`, `SPEED_SCALE_MIN`: how much it slows near
  walls (fraction of base speed).
- Base speeds live in `robot_config.py`: `SPEED_DEFAULT_FORWARD` (150) is the
  main speed knob — raise for a faster run once everything else is trusted.

## 8. Inner-wall bias (OPEN only)  → `config/thresholds.py`

`INNER_WALL_BIAS_PX` (45) shifts the open-challenge racing line toward the inner
wall once direction is known. **0 disables it** (pure centering). Increase for a
tighter line; back off if the car ever brushes the inner wall. Never applied in
an obstacle run.

## 9. Challenge separation & finish  → `config/thresholds.py`

- `OPEN_DECISION_AFTER_CORNERS` (4 = one lap): how long to wait, pillar-free,
  before committing to "this is the Open Challenge." Leave at one lap unless
  obstacle-run pillars appear later than that on your field.
- `START_MOVE_DELAY_S` (0.75): pause after the button so the starter can step
  back. Set `0.0` to launch instantly.
- `FINAL_APPROACH_OPEN_DURATION_S` (1.2): how long the car drives past the
  finish line before stopping on an open run. Tune so it halts **just inside**
  the start section. (Time-based — there is no encoder on this platform.)

## 10. Parallel parking  → `config/parking_config.py`

Tune **durations first**, they dominate. Chalk the spot, run `PARK` repeatedly
(you can force it on the bench by injecting `PARKING_ZONE_DETECTED`).

1. `PARK_SIDE_*` / `PARK_SIDE_OVERRIDE`: confirm the car turns toward the lot.
   If it reverses away from the spot, flip `PARK_SIDE_OVERRIDE`.
2. `PARK_REVERSE_IN_S` (1.30): long enough to swing the tail fully into the
   spot, not so long it hits the far wall.
3. `PARK_REVERSE_STRAIGHTEN_S` (1.20): brings the body parallel to the wall.
4. `PARK_FORWARD_SETTLE_S` (0.45): small nudge off the rear wall.
5. `PARK_STEER_DEG` (70): near full lock — raise for a tighter rotation.
6. Heading guards (`PARK_TURN_IN_DEG`, `PARK_ALIGN_TOLERANCE_DEG`): let the IMU
   end a stage early when the car has rotated/re-aligned enough. Set
   `PARK_USE_HEADING_GUARD = False` to fall back to pure timing if the IMU is
   noisy in the spot.
7. `PARK_MIN_SIDE_CLEARANCE_MM` (60): rear-wall ToF safety cut-off; `0` disables.

---

## Quick reference: the "make it faster" knobs

Once every subsystem above is trusted, these are the levers for a faster run,
in order of impact: `SPEED_DEFAULT_FORWARD` ↑, `SPEED_SCALE_*` ↑ (less braking
near walls), `STEER_SLEW_MAX_DEG_PER_TICK` ↑ (snappier turns),
`START_MOVE_DELAY_S` → 0. Raise one at a time and re-run.
