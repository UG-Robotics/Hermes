"""
Runtime -- the orchestrator that ties every subsystem together into one loop.

Both entry points use it:
    * main.py runs a Runtime directly (real or simulated hardware), and
    * the monitoring dashboard runs a Runtime in a background thread and pokes
      events into it while you watch.

Responsibilities:
    * build the hardware/comms/perception objects (real or simulated),
    * run the 20 Hz control loop:
        vision -> events -> state machine -> drive intent
        -> IMU heading-hold correction -> serial -> telemetry,
    * accept injected state-machine events (from the keyboard, the dashboard,
      a scenario file, or the ESP32's physical start button) and feed them
      through the state machine, and
    * publish a per-tick status snapshot to the TelemetryHub for the dashboard.

Everything it does is logged, so a run produces a complete narrative of what
the robot decided and did -- with real or simulated hardware, identically.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from utils.logger import get_logger
from utils.telemetry_hub import get_hub
from config.robot_config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT

from communication.protocol import serialize_command, get_emergency_packet
from communication.serial_link import SerialLink
from communication.packet_parser import parse_incoming

from control.drive_command import drive_command
from control.steering_control import SteeringController
from state_machine.manager import StateMachine
from state_machine.states import State
from state_machine.event_queue import EventQueue
from state_machine.events import EventType, make_event, EVENT_PRIORITY_MAP

from perception.pillar_detection import PillarDetector
from perception.track_detection import detect_lane
from perception.corner_detection import CornerTracker
from perception.finish_detection import ParkingZoneTracker

from planning.lane_centering import compute_heading_nudge
from planning.obstacle_planner import adjust_avoidance_steer

from config.thresholds import CORNERS_PER_LAP

from hardware.buttons import KeyboardOverrideListener
from hardware.imu import IMU
from hardware.tof import ToFArray
from hardware.diagnostics import run_startup_diagnostics

logger = get_logger("Runtime")

# States in which the pillar-avoidance vision pipeline should run at all.
# Outside these (WAIT_FOR_START, LAP_CHECK, PARK, ...) we don't spend CPU on
# it, and we don't want a stray blob in frame raising a PILLAR_DETECTED event
# somewhere it isn't a valid transition anyway.
_VISION_ACTIVE_STATES = (State.FOLLOW_TRACK, State.AVOID_OBSTACLE, State.LAP_CHECK, State.FINAL_APPROACH)

# Actions for which the IMU heading-hold controller applies a correction.
# STOP means "not moving" -- no point correcting a steer angle for a
# stationary car, and it lets the controller's slew/integral state relax.
_HEADING_HOLD_ACTIONS = ("FORWARD", "BACKWARD")


class Runtime:
    def __init__(self, simulated: bool = False, use_keyboard: bool = True,
                 use_camera: bool = True, loop_hz: float = 20.0,
                 auto_start: bool = False):
        self.simulated = simulated
        self.auto_start = auto_start
        self.loop_interval = 1.0 / loop_hz
        self._hub = get_hub()

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._events = EventQueue()

        # --- comms: real serial or a virtual ESP32 backend --------------------
        backend = None
        if simulated:
            from hardware.sim_esp32 import SimulatedESP32
            backend = SimulatedESP32()
        self.link = SerialLink(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, backend=backend)
        self.serial_ok = self.link.connect()

        # --- sensors ----------------------------------------------------------
        self.imu = IMU(simulated=simulated)
        self.tof = ToFArray(simulated=simulated)

        # --- camera -------------------------------------------------------
        # Built BEFORE the state machine on purpose: item (1) of the pipeline
        # spec requires the camera to be live through the whole of INIT and
        # WAIT_FOR_START so it's ready the instant START_BUTTON_PRESSED
        # arrives. Constructing it here (Runtime.__init__ always runs before
        # StateMachine.__init__ below) guarantees that ordering regardless of
        # which entry point (main.py / dashboard.py) created this Runtime.
        self.camera = None
        if use_camera:
            try:
                from perception.camera import Camera
                self.camera = Camera(simulated=simulated)
            except Exception as e:
                logger.warning(f"Camera unavailable: {e}")
        camera_source = self.camera.source if self.camera else "none"
        # A camera that failed to open when one was requested is a real
        # readiness problem (gate INIT on it). A camera that was never
        # requested at all (use_camera=False -- scenario/headless runs) is a
        # deliberate configuration, not a failure, so it doesn't block INIT.
        camera_ready = (not use_camera) or (self.camera is not None)

        # --- manual driving ---------------------------------------------------
        # Always create the listener so the dashboard can drive remotely even
        # with no physical keyboard; only attach the pynput hook if asked.
        self.listener = KeyboardOverrideListener()
        if use_keyboard:
            try:
                self.listener.start()
            except Exception as e:
                logger.warning(f"Keyboard listener unavailable: {e}")

        # --- decision core ------------------------------------------------
        # camera_ready/serial_ok gate the INIT -> WAIT_FOR_START transition
        # for real: a run with no camera or no ESP32 link now honestly stays
        # in INIT instead of pretending it's ready to race.
        self.state_machine = StateMachine(camera_ready=camera_ready, serial_ready=self.serial_ok)
        self._auto_start_fired = False

        # --- pillar avoidance + IMU heading-hold control -------------------
        self.pillar_detector = PillarDetector()
        self.corner_tracker = CornerTracker()
        self.parking_tracker = ParkingZoneTracker()
        self.steering = SteeringController()

        # The Pi has no way to know the IMU/ToF are alive except by actually
        # seeing a TEL packet arrive from the ESP32 -- a healthy serial link
        # only proves the ESP32 itself is talking, not that ITS sensors came
        # up. Give it a brief window to prove that before we report on it.
        telemetry_confirmed = self._await_first_telemetry() if self.serial_ok else False

        run_startup_diagnostics(simulated, self.serial_ok, camera_source, telemetry_confirmed)
        self._mode_flag = 0
        self._last_tick_ts: Optional[float] = None

    def _await_first_telemetry(self, timeout_s: float = 1.0, poll_interval_s: float = 0.02) -> bool:
        """Block briefly (startup only, never inside tick()) waiting for the
        ESP32's first TEL packet, so run_startup_diagnostics() can report
        real sensor health instead of assuming it from serial_ok alone.

        Any EVT lines that happen to arrive during this window (e.g. the
        start button already being held) are still forwarded through the
        normal handler so nothing gets silently dropped.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self.link.read_line()
            if line:
                parsed = parse_incoming(line)
                if parsed and parsed.get("type") == "event":
                    self._handle_hardware_event(parsed.get("name", ""))
            if self.imu.healthy() and self.tof.healthy():
                return True
            time.sleep(poll_interval_s)
        return self.imu.healthy() and self.tof.healthy()

    # ================================================================== ToF
    def _refresh_tof_context(self) -> None:
        """Publish this tick's freshest ToF reading into RobotContext.

        Called right after _drain_incoming() so control/drive_command.py's
        speed scaling and planning/obstacle_planner.py's avoidance-steer
        capping both see this tick's wall distances, not a stale reading
        from before we last heard from the ESP32. self.tof.read() is a pure
        pass-through over telemetry _drain_incoming() just published (see
        hardware/tof.py's module docstring) -- no hardware access here.
        """
        tof = self.tof.read()
        ctx = self.state_machine.context
        ctx.tof_left_mm = tof["left_mm"]
        ctx.tof_right_mm = tof["right_mm"]

    # ============================================================ event inject
    def inject_event(self, event_type_name: str, source: str = "manual", metadata: Optional[dict] = None) -> bool:
        """Queue a state-machine event by name (e.g. 'START_BUTTON_PRESSED')."""
        try:
            etype = EventType[event_type_name]
        except KeyError:
            logger.error(f"Unknown event type: {event_type_name}")
            return False
        self._events.push(make_event(etype, metadata=metadata or {}))
        self._hub.event(etype.name, EVENT_PRIORITY_MAP[etype].name, source=source)
        logger.info(f"EVENT injected: {etype.name} (source: {source})")
        return True

    def _drain_events(self) -> None:
        """Feed all queued events through the state machine, highest first."""
        for event in self._events.drain():
            old = self.state_machine.current_state
            self.state_machine.handle_event(event)
            new = self.state_machine.current_state
            if new != old:
                self._hub.state(old.name, new.name, via=event.type.name)
            self._post_event_effects(event, old_state=old)
            if new == State.ERROR:
                # Stale integral/target-heading state must not survive into
                # whatever run comes after a manual reset.
                self.steering.reset()

    def _post_event_effects(self, event, old_state) -> None:
        """Bookkeeping the bare state machine doesn't do: record which pillar
        we're avoiding and its computed steer/distance (so drive_command can
        steer around it), lock IMU heading-hold targets, count laps, and
        auto-raise 'three laps complete'.
        """
        ctx = self.state_machine.context
        current_heading = self.imu.heading

        # drive_command() steers around a pillar based on context.last_pillar_color
        # and context.pillar_steer_angle. When the event came from real vision
        # (source="vision", see _update_vision below) metadata carries the
        # actual computed angle/distance; when it came from the keyboard,
        # dashboard, or a scenario file, metadata is empty and we fall back to
        # a sane, correctly-signed default so those entry points still work.
        if event.type == EventType.PILLAR_DETECTED_RED:
            ctx.last_pillar_color = "RED"
            raw_steer = event.metadata.get("steer_angle") or 30
            # TOF-aware cap: don't commit to the full avoidance angle if the
            # wall on the side we're about to swing toward (RIGHT, passing a
            # red pillar) is already close -- see planning/obstacle_planner.py.
            plan = adjust_avoidance_steer(raw_steer, "RIGHT", ctx.tof_left_mm, ctx.tof_right_mm)
            ctx.pillar_steer_angle = plan.steer_angle
            ctx.pillar_distance_mm = event.metadata.get("distance_mm")
            self.steering.turn_by(current_heading, ctx.pillar_steer_angle)
            if plan.wall_nudge_deg:
                self.steering.nudge_target(plan.wall_nudge_deg)
        elif event.type == EventType.PILLAR_DETECTED_GREEN:
            ctx.last_pillar_color = "GREEN"
            raw_steer = event.metadata.get("steer_angle") or -30
            plan = adjust_avoidance_steer(raw_steer, "LEFT", ctx.tof_left_mm, ctx.tof_right_mm)
            ctx.pillar_steer_angle = plan.steer_angle
            ctx.pillar_distance_mm = event.metadata.get("distance_mm")
            self.steering.turn_by(current_heading, ctx.pillar_steer_angle)
            if plan.wall_nudge_deg:
                self.steering.nudge_target(plan.wall_nudge_deg)
        elif event.type == EventType.OBSTACLE_CLEARED:
            ctx.last_pillar_color = None
            ctx.pillar_steer_angle = 0
            ctx.pillar_distance_mm = None
            self.pillar_detector.reset()
            self.steering.hold_straight(current_heading)
        elif event.type == EventType.START_BUTTON_PRESSED:
            # The race just started: lock "straight ahead" from wherever
            # we're pointed right now as the FOLLOW_TRACK baseline.
            self.steering.hold_straight(current_heading)

        if event.type == EventType.LAP_MARKER_DETECTED:
            if old_state == State.FOLLOW_TRACK:
                # Entering LAP_CHECK: this is "one corner started". Count
                # it, and if it's the very first corner marker we've ever
                # seen this run, latch the run's direction from its colour
                # (ORANGE = drive clockwise, BLUE = counter-clockwise --
                # see perception/corner_detection.py's module docstring).
                ctx.corners_passed += 1
                if ctx.race_direction is None:
                    color = event.metadata.get("color")
                    if color == "ORANGE":
                        ctx.race_direction = "CLOCKWISE"
                    elif color == "BLUE":
                        ctx.race_direction = "COUNTER_CLOCKWISE"
                    if ctx.race_direction:
                        logger.info(f"Race direction determined: {ctx.race_direction} "
                                    f"(first corner marker was {color}).")
            elif old_state == State.LAP_CHECK:
                # Exiting LAP_CHECK back to FOLLOW_TRACK: "one corner
                # finished". A full lap is CORNERS_PER_LAP (4) of these.
                if ctx.corners_passed > 0 and ctx.corners_passed % CORNERS_PER_LAP == 0:
                    ctx.increment_lap()
                    if ctx.lap_count >= ctx.target_laps:
                        # The straight section we're already back in IS the
                        # finish zone -- "on the extension of the starting
                        # line segment" per the rules -- so raise both
                        # together; see perception/finish_detection.py's
                        # module docstring for why there's no separate
                        # visual finish-line detector.
                        self._events.push(make_event(EventType.THREE_LAPS_COMPLETE))
                        self._events.push(make_event(EventType.FINISH_ZONE_DETECTED))
                        logger.info("Target laps reached -- raising "
                                    "THREE_LAPS_COMPLETE + FINISH_ZONE_DETECTED.")
        elif event.type == EventType.PARKING_ZONE_DETECTED:
            ctx.parking_detected = True

    # ================================================================= vision
    def _update_vision(self) -> None:
        """Steps (i)-(iii) of the pillar pipeline, run once per tick while
        FOLLOW_TRACK/AVOID_OBSTACLE is active: grab a frame, detect/classify
        any pillar, and raise the appropriate state-machine event. Step (iv)
        (steering angle + distance) is computed inside PillarDetector and
        carried in the event's metadata for _post_event_effects to consume.

        Also dispatches corner-marker tracking (FOLLOW_TRACK + LAP_CHECK)
        and parking-zone tracking (FINAL_APPROACH) off the same captured
        frame -- see _update_corner_tracking()/_update_parking_zone().
        """
        if self.camera is None:
            return
        state = self.state_machine.current_state
        if state not in _VISION_ACTIVE_STATES:
            return

        frame = self.camera.get_frame()
        if frame is None:
            return

        if state in (State.FOLLOW_TRACK, State.AVOID_OBSTACLE):
            obs = self.pillar_detector.update(frame, self.camera.width)

            if state == State.FOLLOW_TRACK:
                # Continuous camera-based corridor centering -- runs every tick
                # regardless of pillar state, nudging the heading-hold target
                # toward the detected wall midpoint. See
                # planning/lane_centering.py's module docstring for why this is
                # a nudge (SteeringController.nudge_target) rather than a
                # turn_by() reset every frame.
                self._update_lane_centering(frame)

                if obs.new_detection:
                    etype = EventType.PILLAR_DETECTED_RED if obs.color == "RED" else EventType.PILLAR_DETECTED_GREEN
                    metadata = {"steer_angle": obs.steer_angle, "distance_mm": obs.distance_mm,
                                "cx": obs.cx, "area": obs.area}
                    self._events.push(make_event(etype, metadata=metadata))
                    self._hub.event(etype.name, EVENT_PRIORITY_MAP[etype].name, source="vision")

            elif state == State.AVOID_OBSTACLE:
                ctx = self.state_machine.context
                if obs.cleared:
                    self._events.push(make_event(EventType.OBSTACLE_CLEARED))
                    self._hub.event(EventType.OBSTACLE_CLEARED.name,
                                     EVENT_PRIORITY_MAP[EventType.OBSTACLE_CLEARED].name, source="vision")
                elif obs.present:
                    # Keep distance/steer telemetry fresh for logging/dashboard
                    # while mid-manoeuvre. We deliberately do NOT re-lock a new
                    # heading target every frame here via turn_by() (that would
                    # reset the PID's integral continuously and produce a
                    # jittery turn) -- the target set once on new_detection is
                    # held until cleared. A side ToF reading a critically close
                    # wall mid-manoeuvre still gets a small corrective nudge
                    # (nudge_target(), not a reset) on top of that held target.
                    if obs.distance_mm is not None:
                        ctx.pillar_distance_mm = obs.distance_mm
                    direction = "RIGHT" if ctx.last_pillar_color == "RED" else "LEFT"
                    plan = adjust_avoidance_steer(ctx.pillar_steer_angle, direction,
                                                   ctx.tof_left_mm, ctx.tof_right_mm)
                    if plan.wall_nudge_deg:
                        self.steering.nudge_target(plan.wall_nudge_deg)

        if state in (State.FOLLOW_TRACK, State.LAP_CHECK):
            self._update_corner_tracking(frame, state)

        if state == State.FINAL_APPROACH:
            self._update_parking_zone(frame)

    def _update_corner_tracking(self, frame, state) -> None:
        """Runs during FOLLOW_TRACK (watching for the next corner to start)
        and LAP_CHECK (watching for the corner we're already in to end).
        Both edges raise the SAME LAP_MARKER_DETECTED event -- see
        perception/corner_detection.py's module docstring for why this
        deliberately mirrors PillarObservation's new_detection/cleared
        pair, and _post_event_effects for how the two edges are told
        apart (by which state the state machine was in when each fired).
        """
        obs = self.corner_tracker.update(frame, self.camera.width)

        fired = (state == State.FOLLOW_TRACK and obs.new_detection) or \
                (state == State.LAP_CHECK and obs.cleared)
        if fired:
            metadata = {"color": obs.color}
            self._events.push(make_event(EventType.LAP_MARKER_DETECTED, metadata=metadata))
            self._hub.event(EventType.LAP_MARKER_DETECTED.name,
                             EVENT_PRIORITY_MAP[EventType.LAP_MARKER_DETECTED].name, source="vision")

    def _update_parking_zone(self, frame) -> None:
        """FINAL_APPROACH only: watch for the obstacle-challenge parking
        lot's magenta boundary. Open-challenge runs simply never see one --
        the mat has no magenta on it -- so this harmlessly never fires and
        FINISH_ZONE_DETECTED (raised from lap-completion bookkeeping, see
        _post_event_effects) is what reaches PARK instead.
        """
        obs = self.parking_tracker.update(frame)
        if obs.confirmed:
            metadata = {"cx": obs.cx, "area": obs.area}
            self._events.push(make_event(EventType.PARKING_ZONE_DETECTED, metadata=metadata))
            self._hub.event(EventType.PARKING_ZONE_DETECTED.name,
                             EVENT_PRIORITY_MAP[EventType.PARKING_ZONE_DETECTED].name, source="vision")

    def _update_lane_centering(self, frame) -> None:
        """One tick of camera-based corridor centering (FOLLOW_TRACK only).

        Detects the corridor's centre offset from the current frame
        (perception/track_detection.py), records it on the context for
        telemetry/dashboard, and -- if the detection is trustworthy enough
        -- nudges the heading-hold controller's locked target toward it.
        """
        obs = detect_lane(frame, self.camera.width, self.camera.height)
        ctx = self.state_machine.context
        ctx.lane_offset_px = obs.offset_px if obs.valid else None
        ctx.lane_confidence = obs.confidence

        nudge = compute_heading_nudge(obs, self.camera.width)
        if nudge:
            self.steering.nudge_target(nudge)

    def _maybe_auto_start(self) -> None:
        if self._auto_start_fired or not self.auto_start:
            return
        if self.state_machine.current_state != State.WAIT_FOR_START:
            return
        self._auto_start_fired = True
        logger.info("Auto-start enabled: injecting START_BUTTON_PRESSED.")
        self.inject_event("START_BUTTON_PRESSED", source="auto-start")

    # ================================================================ hardware events
    def _handle_hardware_event(self, name: str) -> None:
        """An EVT,<name> packet arrived from the ESP32 (e.g. the physical
        start button). Only forward it into the state machine when it's
        actually a valid, expected transition -- this is what makes 'no bot
        movement till the button' hold even if the button is jostled at the
        wrong time (mid-race, before diagnostics pass, etc.)."""
        if not name:
            return
        if name == "START_BUTTON_PRESSED":
            if self.state_machine.current_state == State.WAIT_FOR_START:
                self.inject_event("START_BUTTON_PRESSED", source="button")
            else:
                logger.debug(
                    f"Ignoring START_BUTTON_PRESSED outside WAIT_FOR_START "
                    f"(current state: {self.state_machine.current_state.name})."
                )
        else:
            logger.warning(f"Unhandled hardware EVT from ESP32: {name}")

    # ================================================================= ingest
    def _drain_incoming(self) -> None:
        """PHASE 1 of every tick, and the ONLY place the Pi's view of
        IMU/ToF data is allowed to change.

        The IMU and both ToF sensors are physically wired to the ESP32, not
        the Pi (see hardware/imu.py and hardware/tof.py module docstrings).
        The Pi never opens an I2C bus or reads a sensor register itself --
        it only ever consumes whatever TEL packet the ESP32 (real or
        hardware/sim_esp32.py's virtual one) chooses to send. This method is
        that boundary: every incoming line is parsed here and, if it's
        telemetry, published to the TelemetryHub via parse_incoming() ->
        hub.telemetry(); hardware/imu.py and hardware/tof.py then just read
        the latest published values back out. Nothing downstream of this
        method touches the serial link.

        Runs BEFORE vision/state-machine/steering on purpose: everything
        this tick decides should be based on the freshest data the ESP32
        has sent, not on what's left over from before we last transmitted.
        """
        for _ in range(8):  # drain, don't fall behind a 20Hz telemetry stream
            incoming = self.link.read_line()
            if not incoming:
                break
            parsed = parse_incoming(incoming)
            if parsed and parsed.get("type") == "event":
                self._handle_hardware_event(parsed.get("name", ""))

    # =================================================================== driving
    def _resolve_command(self):
        """Return (speed, steer, action, mode_flag) from manual or autonomous.

        In autonomous mode `steer` is only an *intent* -- straight (0) for
        FOLLOW_TRACK, or a turn-by angle for AVOID_OBSTACLE -- which tick()
        then hands to the IMU heading-hold controller rather than sending
        directly. In manual mode `steer` is the operator's raw command and is
        sent as-is (see tick()).
        """
        manual = self.listener.is_manual_mode_active() if self.listener else False
        if manual:
            speed, steer, action = self.listener.get_manual_target()
            return speed, steer, action, 1
        speed, steer, action = drive_command(
            self.state_machine.current_state, self.state_machine.context
        )
        return speed, steer, action, 0

    def tick(self) -> None:
        """Run exactly one control cycle: ingest -> perceive -> decide -> send.

        Safe to call from a loop or a test.
        """
        now = time.time()
        dt = (now - self._last_tick_ts) if self._last_tick_ts else self.loop_interval
        self._last_tick_ts = now

        # 1) INGEST: pull in everything the ESP32 sent since the last tick --
        #    telemetry (IMU/ToF -- ESP32-owned, see _drain_incoming docstring)
        #    and any hardware events (the start button). Must run first so
        #    every decision below is based on this tick's freshest data.
        self._drain_incoming()

        # 1b) Publish this tick's freshest ToF reading onto the context --
        #     must run before vision/drive-decide so obstacle-planning and
        #     speed scaling below see this tick's wall distances, not a
        #     stale one. See _refresh_tof_context()'s docstring.
        self._refresh_tof_context()

        # 2) PERCEIVE: look for pillars and raise events off real vision.
        self._update_vision()

        # Optional no-button bring-up for bench and integration testing.
        self._maybe_auto_start()

        # 3) STATE MACHINE: apply any events queued this tick (from vision,
        #    keyboard/dashboard injection, a scenario, or the ESP32 button).
        self._drain_events()

        # 4) Pull the IMU view the ingest phase just refreshed. imu.read()
        #    does NOT talk to hardware -- it reads the TelemetryHub record
        #    _drain_incoming() published above, and integrates heading from
        #    the yaw-rate (gz) channel in it. That integration, and the PID
        #    correction in step 6, are Pi-side CONTROL computations over
        #    ESP32-supplied data -- not hardware ownership. See
        #    control/steering_control.py's module docstring for that line.
        imu_reading = self.imu.read()
        ctx = self.state_machine.context
        ctx.heading = imu_reading["heading_deg"]

        # 5) DECIDE what we intend to do (speed/action always; steer is only
        #    an intent in autonomous mode -- see _resolve_command docstring).
        speed, intent_steer, action, mode_flag = self._resolve_command()
        just_left_manual = (mode_flag == 0 and self._mode_flag == 1)
        if mode_flag != self._mode_flag:
            self._hub.mode(bool(mode_flag))
            self._mode_flag = mode_flag

        # 6) IMU heading-hold correction -- item (3) of the spec: "the IMU
        #    should always act as a PID... checking the orientation... and
        #    turning the correct angle... communicate with the servo to do
        #    these corrections smoothly." This is that loop. It only runs in
        #    autonomous mode while actually driving; manual driving is left
        #    un-corrected (operator has direct control) and the controller is
        #    reset so no stale state leaks back in on the next auto tick.
        heading_error = 0.0
        target_heading = self.steering.target_heading_deg
        if mode_flag == 0 and action in _HEADING_HOLD_ACTIONS:
            if just_left_manual:
                self.steering.reset()
            steer, heading_error = self.steering.compute(ctx.heading, dt)
            target_heading = self.steering.target_heading_deg
        elif mode_flag == 1:
            steer = intent_steer  # raw operator command, sent as-is
        else:
            # Autonomous but stopped (STOP/ERROR/INIT/WAIT_FOR_START/PARK-idle):
            # relax the controller so it doesn't fire a correction the instant
            # we start moving again.
            self.steering.reset()
            steer = 0

        # Record what we're commanding, in structured form and as a human line.
        source = "manual" if mode_flag else "auto"
        self._hub.command(speed, steer, action, source)
        ctx.speed, ctx.steering = speed, steer

        # 7) SEND to the ESP32 (real or virtual) -- step (v) of the pillar
        #    pipeline and the output of the IMU heading-hold loop both land
        #    here, on the wire, identically to any other steer command. This
        #    tick's response to what we just ingested will be picked up by
        #    _drain_incoming() at the top of the NEXT tick, not this one --
        #    the Pi never blocks waiting for an immediate reply.
        packet = serialize_command(speed, steer, action, mode_flag)
        self.link.send(packet)

        # One tidy status record the dashboard renders as the "current state".
        # tof_left_mm/right_mm were already refreshed this tick by
        # _refresh_tof_context() (step 1b), so this is just re-reading the
        # context, not the hardware.
        self._hub.publish("status", {
            "state": self.state_machine.current_state.name,
            "mode": "MANUAL" if mode_flag else "AUTO",
            "speed": speed, "steer": steer, "action": action,
            "lap": ctx.lap_count, "target_laps": ctx.target_laps,
            "heading_deg": round(imu_reading["heading_deg"], 1),
            "target_heading_deg": None if target_heading is None else round(target_heading, 1),
            "heading_error_deg": round(heading_error, 1),
            "imu": {k: round(v, 3) for k, v in imu_reading.items() if k != "heading_deg"},
            "tof": {"left_mm": round(ctx.tof_left_mm, 1), "right_mm": round(ctx.tof_right_mm, 1)},
            "lane": {"offset_px": ctx.lane_offset_px,
                     "confidence": round(ctx.lane_confidence, 2)},
            "corner": {"corners_passed": ctx.corners_passed,
                       "race_direction": ctx.race_direction,
                       "parking_detected": ctx.parking_detected},
            "pillar": {"color": ctx.last_pillar_color, "steer_angle": ctx.pillar_steer_angle,
                       "distance_mm": ctx.pillar_distance_mm},
            "simulated": self.simulated,
        })

    # ==================================================================== loops
    def run_forever(self) -> None:
        """Blocking run loop (used by main.py)."""
        self._running = True
        logger.info(
            "\n\n==============================================="
            "\nHermes Autonomous Vehicle Software Starting..."
            f"\nMode: {'SIMULATION' if self.simulated else 'REAL HARDWARE'}"
            "\n==============================================="
        )
        logger.info("Manual controls: w/s drive, a/d steer, space stop, m toggle.")
        try:
            while self._running:
                start = time.time()
                self.tick()
                elapsed = time.time() - start
                time.sleep(max(0.0, self.loop_interval - elapsed))
        except KeyboardInterrupt:
            logger.info("Shutdown requested.")
        except Exception as run_err:
            logger.error(f"Runtime error: {run_err}", exc_info=True)
        finally:
            self.shutdown()

    def start_background(self) -> None:
        """Run the loop in a daemon thread (used by the dashboard)."""
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._bg_loop, daemon=True, name="RuntimeLoop")
        self._thread.start()
        logger.info("Runtime loop started in background.")

    def _bg_loop(self) -> None:
        try:
            while self._running:
                start = time.time()
                self.tick()
                elapsed = time.time() - start
                time.sleep(max(0.0, self.loop_interval - elapsed))
        except Exception as e:
            logger.error(f"Background runtime error: {e}", exc_info=True)

    def stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        logger.warning("Triggering safety shutdown.")
        self._running = False
        try:
            self.link.send_emergency(get_emergency_packet(mode=1))
        except Exception as e:
            logger.error(f"Emergency send failed: {e}")
        if self.listener:
            self.listener.stop()
        if self.camera:
            self.camera.close()
        logger.info("Shutdown completed.")
