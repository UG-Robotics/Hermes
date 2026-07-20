# `src/` - Robot Runtime Software

## Overview

This directory contains the complete runtime software stack for the autonomous vehicle used in the WRO Future Engineers competition.

The purpose of this folder is to organize all software required for:

* autonomous navigation
* perception and obstacle detection
* decision-making
* motion planning
* low-level control
* communication with embedded hardware

This directory intentionally contains ONLY software directly related to the operation of the physical robot during runtime.

In practice, that includes the live monitoring dashboard, state-machine logic, testing assets, and the embedded controller firmware alongside the main autonomy loop.

The following are intentionally kept out of the runtime path:

* CAD files
* documentation assets (kept in the repo-root folders)
* temporary experiments

Development-only tooling that *does* live under `src/` is clearly separated
from the runtime stack: `simulation/` (Gazebo world + MATLAB vision notes) and
`testing/` are for the bench, not the competition loop, and nothing in the
autonomy path imports them.

---

# High-Level System Architecture

The software stack follows a layered robotics architecture:

```text
PERCEPTION
    ↓
EVENT GENERATION
    ↓
STATE MACHINE / DECISION LOGIC
    ↓
MOTION PLANNING
    ↓
CONTROL
    ↓
COMMUNICATION
    ↓
MCU → MOTOR + SERVO ACTUATION
```

The Raspberry Pi acts as the high-level autonomy computer:

* computer vision
* state transitions
* planning
* decision-making

The ESP32 acts as the low-level controller:

* PWM generation
* steering actuation
* motor actuation
* real-time control loops

---

# Directory Structure

```text
src/
│
├── main.py                 # CLI entry point / mode dispatcher
├── runtime.py              # the orchestrator: the 20 Hz control loop
│
├── perception/
├── state_machine/
├── planning/
├── control/
├── communication/
├── hardware/
├── monitoring/
├── testing/
├── firmware/
├── simulation/             # Gazebo / MATLAB assets (dev only, not runtime)
├── utils/
└── config/
```

---

# Entry Point

## `main.py`

The thin command-line front door. It parses `--mode` and hardware flags
(`--real` / `--sim`) and hands off to the right runner:

* `run` — real hardware, the competition entry point (default)
* `sim` — the same loop against a simulated ESP32 + synthetic camera
* `dashboard` — launch the live monitoring web UI
* `scenario` — replay a scripted event scenario headless
* `test` — run the unittest suite

`main.py` holds no business logic — it only wires up a `Runtime` (or a
scenario/test runner) and starts it.

## `runtime.py`

The actual orchestrator. `Runtime` builds every subsystem (real or simulated),
then runs one control cycle per tick at ~20 Hz:

```text
ingest ESP32 telemetry (IMU / ToF)      ← _drain_incoming
    ↓
perceive (camera → pillar/lane/corner/parking events)   ← _update_vision
    ↓
update state machine (drain queued events)   ← _drain_events
    ↓
decide drive intent (state → speed/steer/action)   ← _resolve_command
    ↓
IMU heading-hold PID correction
    ↓
send CMD packet to ESP32
    ↓
publish a telemetry snapshot for the dashboard
```

It is also where run-level behaviour lives that isn't any single subsystem's
job: challenge auto-detection (OPEN vs OBSTACLE), lap counting, the start
delay, the open-run finish timer, and driving the parking maneuver. Keep
per-subsystem logic in the modules below; `runtime.py` only coordinates them.

---

# Module Descriptions

---

# `perception/`

Responsible for environmental understanding and sensor interpretation.

This module processes camera and sensor data to generate meaningful observations for the rest of the system.

## Responsibilities

* image acquisition
* preprocessing
* ROI extraction
* track detection
* wall detection
* obstacle detection
* pillar color classification
* finish zone detection

## Example Files

```text
perception/
├── camera.py            # real (Picamera2/OpenCV) + synthetic frame source
├── preprocessing.py     # thin compatibility wrapper over pillar detection
├── roi.py               # region-of-interest crop helpers
├── track_detection.py   # wall/corridor detection (lane centering)
├── pillar_detection.py  # red/green traffic-sign pillars + pass-side
├── corner_detection.py  # orange/blue corner markers → lap + direction
└── finish_detection.py  # magenta parking-zone marker
```

## Notes

This module should NOT:

* control motors
* make navigation decisions
* contain PID logic

It only answers:

> “What does the robot currently observe?”

---

# `state_machine/`

Responsible for robot behavioural logic.

This module determines:

* what state the robot is currently in
* when state transitions occur
* how events are prioritized

## Responsibilities

* state tracking
* transition logic
* event handling
* state priority management
* fail-safe behaviour

## Example Files

```text
state_machine/
├── states.py         # the State enum
├── transitions.py    # (state, event) → next state table
├── events.py         # EventType enum + priority map + Event/make_event
├── priorities.py     # Priority levels
├── event_queue.py    # thread-safe priority queue (drain per tick)
├── robot_context.py  # shared run state (laps, direction, challenge_mode, …)
└── manager.py        # applies events, gates INIT on camera/serial readiness
```

## Example States

```text
INIT
WAIT_FOR_START
FOLLOW_TRACK
AVOID_OBSTACLE
LAP_CHECK
FINAL_APPROACH
PARK
STOP
ERROR
```

## Notes

The state machine should only manage behaviour and transitions.

It should NOT:

* process raw camera frames
* directly generate PWM signals

---

# `monitoring/`

Responsible for the live dashboard, scenario runner, and telemetry visualization used during development and testing.

This module mirrors the same telemetry stream used by the robot runtime, so the dashboard and the headless scenario runner always show the current system state.

## Responsibilities

* dashboard UI
* scenario simulation
* telemetry visualization
* manual control hooks
* live log display

## Example Files

```text
monitoring/
├── dashboard.py
├── scenario.py
├── __init__.py
├── scenarios/
└── templates/
```

## Notes

The full module documentation lives in `monitoring/README.md`.

---

# `testing/`

Responsible for automated regression tests and mock event helpers.

This folder keeps verification assets separate from the runtime stack while still documenting the expected behaviour of the major software subsystems.

## Responsibilities

* unit tests
* protocol validation
* state-machine regression coverage
* robot-context checks
* mock event generation

## Example Files

```text
testing/
├── test_drive_command.py
├── test_event_queue.py
├── test_protocol.py
├── test_robot_context.py
├── test_transitions.py
└── mock_events.py
```

---

# `firmware/esp_controller/`

Responsible for the microcontroller firmware that executes low-level motor, servo, and telemetry tasks.

This firmware acts as the bridge between high-level autonomy code and the physical actuators on the robot.

## Responsibilities

* serial protocol handling
* motor control
* servo control
* telemetry reporting
* embedded configuration

## Example Files

```text
firmware/esp_controller/
├── esp_controller.ino
├── config.h
├── motor.cpp
├── motor.h
├── servo_control.cpp
├── servo_control.h
├── serial_protocol.cpp
├── serial_protocol.h
├── telemetry.cpp
├── telemetry.h
└── robot_data.h
```

---

# `planning/`

Responsible for motion-level decision making.

This layer transforms behavioural goals into motion targets.

## Responsibilities

* lane centering
* obstacle-side planning
* steering target generation
* parking trajectory generation
* path adjustment

## Example Files

```text
planning/
├── lane_centering.py
├── obstacle_planner.py
├── trajectory.py
└── parking_planner.py
```

## Notes

This layer answers:

> “Where should the robot move next?”

---

# `control/`

Responsible for low-level motion stabilization and command generation.

This module converts motion targets into stable control outputs.

## Responsibilities

* PID control
* steering smoothing
* speed regulation
* filtering
* actuator command generation

## Example Files

```text
control/
├── pid.py
├── steering_control.py
├── speed_controller.py
└── filters.py
```

## Notes

This module should NOT:

* perform computer vision
* make strategic decisions

It answers:

> “How do we physically achieve the desired motion?”

---

# `communication/`

Responsible for Raspberry Pi ↔ MCU communication.

This layer abstracts the serial communication protocol between the high-level computer and the embedded controller.

## Responsibilities

* serial communication
* packet encoding
* packet decoding
* command validation
* timeout handling

## Example Files

```text
communication/
├── serial_link.py
├── protocol.py
└── packet_parser.py
```

## Wire Protocol

Every line is a self-identifying, comma-delimited packet (see
`protocol.py` / `packet_parser.py`).

Pi → ESP32:

```text
CMD,<speed>,<steer>,<ACTION>,<mode>   e.g. CMD,150,-12,FORWARD,0
EMG,<mode>                            emergency stop (bypasses CMD parsing)
PING                                  liveness check
```

ESP32 → Pi:

```text
TEL,ax,ay,az,gx,gy,gz,tof1_mm,tof2_mm   sensor telemetry (~20 Hz)
STATUS,<message>                        boot / error text
EVT,<name>                              hardware event, e.g. START_BUTTON_PRESSED
ACK[,tag]                               acknowledgement
```

## Notes

Communication failures should trigger safe fail-state behaviour.

---

# `hardware/`

Responsible for hardware abstraction and sensor interfaces.

This layer isolates hardware-specific implementations from higher-level logic.

## Responsibilities

* IMU interface
* TOF sensor interface
* button input handling
* diagnostics
* hardware initialization

## Example Files

```text
hardware/
├── imu.py
├── tof.py
├── buttons.py
└── diagnostics.py
```

## Notes

This layer should expose clean interfaces to the rest of the system.

Higher-level modules should not depend on raw hardware implementation details.

---

# `utils/`

Contains reusable helper utilities used across the project.

## Responsibilities

* logging
* timers
* debugging utilities
* math helper functions
* common reusable helpers

## Example Files

```text
utils/
├── logger.py
├── timers.py
├── math_utils.py
└── debug.py
```

---
#`utils/logger.py`
# Logging System

## Purpose

The logging system provides a consistent way for every subsystem of the robot to report information, warnings, errors, and debugging information during development, simulation, and competition runs.

Examples of information that should be logged include:

* State transitions
* Sensor readings
* Obstacle detections
* Lap completions
* Parking events
* Communication failures
* Runtime errors
* Emergency stops

Using a centralized logger makes debugging significantly easier because all messages follow the same format and can be traced back to their originating module.

---

## Logger Location

```text
src/
└── utils/
    └── logger.py
```

---

## Implementation

The logger utility exposes a single function:

```python
from utils.logger import get_logger
```

Example implementation:

```python
logger = get_logger(__name__)
```

The `__name__` parameter automatically identifies the module generating log messages.

For example:

```python
logger = get_logger(__name__)
```

inside:

```text
state_machine/robot_context.py
```

will produce messages such as:

```text
[INFO] state_machine.robot_context: Lap completed. Current lap count = 1
```

---

## Log Levels

### INFO

Used for normal system operation.

Examples:

```python
logger.info("Lap completed.")
logger.info("State transition: INIT -> FOLLOW_TRACK")
```

Output:

```text
[INFO] state_machine.robot_context: Lap completed.
```

---

### WARNING

Used for unusual but recoverable situations.

Examples:

```python
logger.warning("TOF reading appears noisy.")
logger.warning("RobotContext reset.")
```

Output:

```text
[WARNING] state_machine.robot_context: RobotContext reset.
```

---

### ERROR

Used when a subsystem encounters a problem.

Examples:

```python
logger.error("Camera initialization failed.")
logger.error("Robot Error: Camera Failure")
```

Output:

```text
[ERROR] state_machine.robot_context: Robot Error: Camera Failure
```

---

## Usage Rules

Every new module should create its own logger instance.

Example:

```python
from utils.logger import get_logger

logger = get_logger(__name__)
```

Recommended locations:

```text
communication/
control/
hardware/
perception/
planning/
simulation/
state_machine/
```

Do NOT create custom logging implementations in individual modules.

All logging should go through `utils.logger`.

---

## Good Logging Examples

State machine:

```python
logger.info(
    f"State transition: {old_state} -> {new_state}"
)
```

Obstacle detection:

```python
logger.info(
    f"Red pillar detected at x={pillar_x}"
)
```

Communication:

```python
logger.warning(
    "Serial packet dropped."
)
```

Error handling:

```python
logger.error(
    "Camera disconnected."
)
```

---

## Bad Logging Examples

Avoid:

```python
print("something happened")
```

Use:

```python
logger.info("Something happened.")
```

This ensures consistent formatting across the entire project.


---

# `config/`

Contains all configurable runtime parameters.

Centralizing parameters improves:

* maintainability
* tuning
* testing
* reproducibility

## Responsibilities

* PID constants
* camera thresholds
* steering limits
* robot dimensions
* sensor thresholds

## Example Files

```text
config/
├── pid_config.py       # heading-hold PID gains + servo slew
├── thresholds.py       # lane/ToF/speed/lap/challenge/start-delay constants
├── camera_config.py    # resolution + HSV colour ranges + blob/distance
├── parking_config.py   # parallel-parking maneuver timings + geometry
└── robot_config.py     # serial, speeds, steering limits, pins, keymap
```

## Notes

Magic numbers should NEVER be hardcoded throughout the codebase.

Configuration values should be stored here whenever possible.

The full on-mat tuning procedure for these constants lives in
[`CALIBRATION.md`](CALIBRATION.md).

---

# Design Philosophy

The software architecture follows several engineering principles:

## 1. Separation of Concerns

Each module has a single clear responsibility.

Examples:

* perception detects
* planning decides
* control stabilizes
* communication transmits

This prevents tightly coupled “spaghetti code”.

---

## 2. Modularity

Modules should be independently testable and replaceable.

For example:

* obstacle detection can be improved without rewriting the state machine
* PID tuning can change without affecting perception

---

## 3. Deterministic Behaviour

The system prioritizes:

* reliability
* repeatability
* explainability

The architecture is intentionally designed around deterministic robotics principles rather than heavy machine learning pipelines.

---

## 4. Safety and Fail-Safe Design

Critical failures should immediately trigger:

* motor stop
* transition to ERROR state
* safe shutdown behaviour

Examples:

* camera disconnect
* serial timeout
* invalid sensor data

---

# Coding Guidelines

## General Rules

* Keep functions small and focused
* Avoid giant monolithic files
* Prefer readability over cleverness
* Use descriptive variable names
* Document assumptions clearly

---

## File Responsibilities

A file should ideally answer ONE question.

Bad example:

```text
vision_and_pid_and_serial.py
```

Good examples:

```text
pillar_detection.py
serial_link.py
pid.py
```

---

## Logging

Important events should be logged:

* state transitions
* obstacle detections
* communication failures
* lap completion

This improves debugging and post-run analysis.

---

# Runtime Flow Summary

The expected runtime execution order is:

```text
1. Initialize systems
2. Wait for official start
3. Begin track following
4. Detect and avoid obstacles
5. Count laps
6. Return to final section
7. Park / stop
8. Enter safe idle state
```

---

# Intended Audience

This folder structure and architecture are designed to be understandable by:

* judges
* teammates
* future contributors
* reviewers
* maintainers

The project prioritizes:

* clarity
* robustness
* maintainability
* engineering discipline

over unnecessary complexity.

---
<!-- 
# Future Extensions

Potential future improvements may include:

* sensor fusion
* advanced trajectory planning
* adaptive PID tuning
* lightweight ML-assisted perception
* telemetry logging
* improved simulation integration -->

The current architecture is intentionally designed to support future expansion without major restructuring.

---

# Final Notes

This software architecture represents a deliberate engineering decision to prioritize:

* deterministic behaviour
* modularity
* reliability
* maintainability
* competition robustness

The architecture is structured to allow incremental development, independent subsystem testing, and clear separation between perception, decision-making, and control.
