# `monitoring/` — Dashboard & Simulation

A single web page to watch and drive the whole robot, plus a scenario runner for
scripted simulations. Everything here reads from the **TelemetryHub**
(`utils/telemetry_hub.py`), so it shows the exact same data whether the robot is
running on real hardware or fully simulated.

## Quick start

```bash
cd src

# 1. Live monitoring dashboard, fully simulated (no robot needed):
python main.py --mode dashboard
#   → open http://<this-machine>:5000

# 2. Same, but driving REAL hardware from the browser:
python main.py --mode dashboard --real

# 3. Headless simulation of a scripted scenario (prints the captured log):
python main.py --mode scenario --scenario full_match
```

Dependencies for the dashboard: `pip install -r ../requirements.txt` (Flask +
Pillow). The control loop and scenarios run on the standard library alone.

## What the dashboard shows

| Panel | Source channel | Notes |
|-------|----------------|-------|
| Camera | `camera.mjpg` | Pi Camera / webcam if present, else a synthetic drawn corridor |
| IMU (ax…gz) | `telemetry` | live 6-axis, real or simulated |
| ToF (left/right) | `telemetry` | dual distance sensors |
| State + Action/Speed/Steer/Lap | `status` | current state-machine state |
| Manual Driving | `POST /api/manual` | on-screen pad + in-browser w/s/a/d keys |
| Inject Events | `POST /api/event` | fire any state-machine event by hand |
| Scenario Simulation | `POST /api/scenarios` | run a scripted event string |
| Pi ⇄ ESP32 Serial | `comms` | live TX/RX packets crossing the link |
| Live Log | `log` | every `logger.*` call in the whole codebase |

## Scenarios ("string events together")

A scenario is a JSON file in `scenarios/` listing timed events:

```json
{
  "name": "obstacle_avoid",
  "description": "Start, dodge a red pillar, resume.",
  "events": [
    {"at": 0.5, "event": "START_BUTTON_PRESSED"},
    {"at": 2.0, "event": "PILLAR_DETECTED_RED"},
    {"at": 4.0, "event": "OBSTACLE_CLEARED"}
  ]
}
```

`at` is seconds from the scenario start. Valid event names are the members of
`EventType` in `state_machine/events.py`. Drop a new `.json` in `scenarios/` and
it appears in the dashboard dropdown automatically.

Run one from the dashboard (dropdown → **Run Scenario**), or headless:

```bash
python main.py --mode scenario --scenario obstacle_avoid
```

## HTTP API

| Method | Path | Body | Purpose |
|--------|------|------|---------|
| GET | `/api/snapshot` | — | latest value on every channel |
| GET | `/api/logs?limit=200` | — | recent log records |
| GET | `/stream` | — | SSE stream of all channels |
| GET | `/camera.mjpg` | — | MJPEG camera feed |
| POST | `/api/event` | `{"event": "LAP_MARKER_DETECTED"}` | inject an event |
| POST | `/api/manual` | `{"active": true, "target": {"speed":150,"steer":-45,"action":"FORWARD"}}` | manual drive |
| GET/POST | `/api/scenarios` | `{"action":"run","scenario":"full_match"}` | list / run / cancel |
