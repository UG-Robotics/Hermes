# HERMES - WRO 2026 Future Engineers 

Hermes is an autonomous 1:X model car built for the **World Robot Olympiad
2026 Future Engineers** self-driving challenge. It completes both events on the
3m × 3m field — the **Open Challenge** (three laps of a randomised-wall track)
and the **Obstacle Challenge** (three laps obeying red/green traffic signs, then
a parallel park) — under a single start button, with no human input after the
press.

This repository is the complete engineering record: mechanical CAD
(`models/`), electronics and wiring (`schemes/`, `components/`), the full
robot software (`src/`), and supporting media (`v-photos/`, `t-photos/`,
`video/`). The detailed build write-up is in
[`HERMES_TechnicalDocument_WRO2026.pdf`](HERMES_TechnicalDocument_WRO2026.pdf).

---

## How it works at a glance

Hermes uses a **two-tier compute** split, connected by one UART link:

| Tier | Board | Responsibility |
|---|---|---|
| **High-level autonomy** | Raspberry Pi 4 | camera vision, state machine, planning, control decisions |
| **Low-level real-time** | ESP32 | motor + steering-servo PWM, IMU + dual ToF sensing, start button, telemetry |

The Pi never touches a sensor register directly — every IMU/ToF value arrives
as a telemetry packet from the ESP32, and every drive decision leaves as a
command packet. That clean boundary is what lets the **entire stack run on a
laptop with no robot attached** (simulated ESP32 + synthetic camera) for
development and testing.

**Core hardware:** Raspberry Pi 4 · ESP32 · Pi Camera · LSM6DSO 6-axis IMU ·
2× VL53L0X time-of-flight sensors · N20 gear-motor with L298N driver ·
steering servo (Ackermann) · 18650 Li-ion pack + buck converter. See
[`components/`](components/) and [`schemes/`](schemes/) for parts and wiring.

---

## Challenge strategy

### One button, two challenges (auto-detection)

WRO rules allow only **one** start button, so Hermes can't be *told* which
challenge it's running — it works it out itself. The insight: a **red/green
traffic-sign pillar is unique to the Obstacle Challenge**.

- See a pillar → latch **Obstacle** mode (drive the pass-side + parking logic).
- Complete a **full lap with no pillar** → latch **Open** mode (drop pillar
  perception, bias toward the inner wall, finish by lap count).

### Open Challenge

Drive three laps of the corridor using **camera-based wall centering** trimmed
by an **IMU heading-hold PID**, counting the four orange/blue corner markers per
lap. The colour of the *first* corner marker latches the run **direction**;
Hermes then biases its racing line toward the **inner wall** for a tighter,
safer line. After the third lap it clears the finish line and stops.

### Obstacle Challenge

Same lap-following, plus: each red/green pillar raises an avoidance turn —
**pass right of red, left of green** — executed as a smooth IMU-held heading
change, and capped by the side ToF sensors so clearing a pillar never means
clipping a wall. After three laps it hunts the **magenta parking lot** and runs
a staged **parallel-parking** maneuver.

The full behaviour is a priority-driven state machine
(`INIT → WAIT_FOR_START → FOLLOW_TRACK ⇄ AVOID_OBSTACLE / LAP_CHECK →
FINAL_APPROACH → PARK → STOP`, with `ERROR` fail-safe). Diagram and spec:
[`other/State_Transition_Diagram.png`](other/State_Transition_Diagram.png),
[`other/State_Machine_Spec.pdf`](other/State_Machine_Spec.pdf).

---

## Running the software

All commands run from `src/`. The control loop and simulation need only the
**standard library**; the extras below unlock hardware, the dashboard, and
vision.

```bash
pip install -r requirements.txt      # flask, pyserial, pynput, pillow, numpy
pip install opencv-python            # required for real camera perception

python main.py --mode sim --auto-start   # full stack vs simulated hardware, no robot
python main.py --mode dashboard          # live web UI (telemetry + camera + manual drive)
python main.py --mode test               # unit + integration test suite
python main.py --mode run                # REAL hardware (Pi + ESP32) — the competition entry
```

Manual driving is always available (keyboard `w/a/s/d`, space, `m` to toggle, or
the dashboard controls) as an override. Firmware for the ESP32 lives in
[`src/firmware/esp_controller/`](src/firmware/esp_controller/) (Arduino).

---

## Tuning for competition day

The software is written so that going from "runs" to "wins" is **only constant
tuning** — no code changes. Every threshold, gain, colour range, and maneuver
timing is centralised in [`src/config/`](src/config/), and the step-by-step
on-mat procedure (servo trim, IMU sign, PID, HSV colours, ToF safety, parking
timings, …) is documented in **[`src/CALIBRATION.md`](src/CALIBRATION.md)**.

---

## Repository map

| Path | Contents |
|---|---|
| [`src/`](src/) | All robot software — see [`src/README.md`](src/README.md) for the module-by-module breakdown |
| [`src/config/`](src/config/) | Every tunable constant (camera, PID, thresholds, parking, robot) |
| [`src/firmware/`](src/firmware/) | ESP32 real-time controller firmware |
| [`models/`](models/) | Mechanical CAD (chassis, steering, drivetrain) |
| [`schemes/`](schemes/) | Wiring diagrams and electronics schematics |
| [`components/`](components/) | Bill of materials, datasheets, part photos |
| [`v-photos/`](v-photos/) · [`t-photos/`](t-photos/) | Vehicle and team photos |
| [`video/`](video/) | Performance video links ([`video/README.md`](video/README.md)) |
| [`other/`](other/) | State-machine spec and diagrams |

---

## Software architecture

Layered robotics stack, one direction of data flow, each layer independently
testable:

```
PERCEPTION  →  EVENTS  →  STATE MACHINE  →  PLANNING  →  CONTROL  →  COMMUNICATION  →  ESP32
 (camera)     (typed,     (priority        (lane /      (heading-    (UART packets)    (motor +
              prioritised) transitions)     obstacle /   hold PID +                     servo +
                                            parking)     slew)                          sensors)
```

Perception detects, planning decides, control stabilises, communication
transmits — no layer reaches across another. A live **monitoring dashboard**
(`src/monitoring/`) mirrors the same telemetry the robot runs on, so a run
produces a complete, replayable narrative of every decision. Full rationale and
per-module docs: [`src/README.md`](src/README.md).

