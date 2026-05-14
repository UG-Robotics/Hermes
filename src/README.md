# `src/` - Robot Runtime Software

## Overview

This directory contains the complete runtime software stack for the autonomous vehicle used by University of Ghana Team B (HERMES) in the WRO Future Engineers competition.

The purpose of this folder is to organize all software required for:

* autonomous navigation
* perception and obstacle detection
* decision-making
* motion planning
* low-level control
* communication with embedded hardware

This directory intentionally contains ONLY software directly related to the operation of the physical robot during runtime.

The following are intentionally excluded from this folder:

* simulation environments
* Gazebo projects
* MATLAB models
* testing notebooks
* CAD files
* documentation assets
* temporary experiments

Those components are stored outside this directory for clarity and maintainability.

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

The STM32 / Arduino Nano acts as the low-level controller:

* PWM generation
* steering actuation
* motor actuation
* real-time control loops

---

# Directory Structure

```text
src/
│
├── main.py
│
├── perception/
├── state_machine/
├── planning/
├── control/
├── communication/
├── hardware/
├── utils/
└── config/
```

---

# Entry Point

## `main.py`

Main runtime execution loop for the robot.

Responsibilities:

* initialize all subsystems
* initialize sensors
* initialize serial communication
* initialize state machine
* execute main control loop
* coordinate perception, planning, and control

The main runtime loop follows the architecture:

```text
capture sensor data
    ↓
generate events
    ↓
update state machine
    ↓
compute motion targets
    ↓
run controllers
    ↓
send commands to MCU
```

`main.py` should remain lightweight and orchestration-focused.

Business logic should NOT be implemented directly inside `main.py`.

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
├── camera.py
├── preprocessing.py
├── roi.py
├── track_detection.py
├── pillar_detection.py
└── finish_detection.py
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
├── states.py
├── transitions.py
├── events.py
├── priorities.py
└── manager.py
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

## Example Commands

```text
SPEED,40
STEER,-12
STOP
MODE,1
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
├── pid_config.py
├── thresholds.py
├── camera_config.py
└── robot_config.py
```

## Notes

Magic numbers should NEVER be hardcoded throughout the codebase.

Configuration values should be stored here whenever possible.

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
