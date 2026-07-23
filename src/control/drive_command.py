from state_machine.states import State
from state_machine.robot_context import RobotContext
from config.robot_config import (
    SPEED_DEFAULT_FORWARD,
    SPEED_DEFAULT_BACKWARD,
    SPEED_STOP,
    STEER_CENTER_DEGREE,
)
from planning.obstacle_planner import general_speed_scale
from control.speed_controller import apply_speed_scale
from utils.logger import get_logger

logger = get_logger(__name__)


def drive_command(state: State, context: RobotContext) -> tuple[int, int, str]:
    """
    Maps the current robot state and context to a (speed, steering, action) tuple.
    This is the autonomous drive decision layer — called every loop tick.

    Returns:
        speed   (int): 0–255
        steer   (int): -90 to 90 (negative = left, positive = right)
        action  (str): 'FORWARD' | 'BACKWARD' | 'STOP'
    """

    if state == State.FOLLOW_TRACK:
        # TOF wall-clearance speed scaling: ease off if a side wall is
        # close (tight corner, narrow 600mm open-challenge corridor, etc.)
        # -- see planning/obstacle_planner.py. context.tof_left_mm/right_mm
        # are refreshed every tick in runtime.py right after telemetry
        # ingest, so this always sees this tick's freshest wall distances.
        scale = general_speed_scale(context.tof_left_mm, context.tof_right_mm)
        speed = apply_speed_scale(SPEED_DEFAULT_FORWARD, scale)
        # This is only the intent handed to the IMU heading-hold controller
        # (control/steering_control.py) in runtime.py, which locks "straight
        # from here" (nudged toward the corridor centre from the side ToFs by
        # planning/wall_centering.py) and does the actual closed-loop
        # correction, not the final steer value sent to the ESP32.
        steer = 0
        action = "FORWARD"
        logger.debug(f"[DRIVE] FOLLOW_TRACK -> speed={speed} (scale={scale:.2f}), steer intent={steer}")

    elif state == State.AVOID_OBSTACLE:
        # Competition rule: pass RIGHT of RED pillars, LEFT of GREEN pillars.
        # steer is a turn-by intent in degrees (+ = right, - = left), fed to
        # the IMU heading-hold controller in runtime.py, not sent raw.
        # context.pillar_steer_angle is set by the vision pipeline
        # (perception/pillar_detection.py via runtime.py) when a real pillar
        # was seen; it falls back to a sane, correctly-signed default if this
        # state was entered via a manually-injected/scenario event with no
        # vision metadata attached.
        if context.pillar_steer_angle:
            steer = context.pillar_steer_angle
        elif context.last_pillar_color == "RED":
            steer = 30   # pass on the right
        elif context.last_pillar_color == "GREEN":
            steer = -30  # pass on the left
        else:
            steer = 0
        # Same wall-clearance speed scaling as FOLLOW_TRACK. The steer angle
        # itself is ALREADY capped for wall proximity by
        # planning.obstacle_planner.adjust_avoidance_steer() at the point
        # runtime.py locks it via context.pillar_steer_angle (see
        # runtime.py's _post_event_effects) -- this is only the speed term.
        scale = general_speed_scale(context.tof_left_mm, context.tof_right_mm)
        speed = apply_speed_scale(SPEED_DEFAULT_FORWARD, scale)
        action = "FORWARD"
        logger.debug(
            f"[DRIVE] AVOID_OBSTACLE -> pillar={context.last_pillar_color}, "
            f"steer intent={steer}, speed={speed} (scale={scale:.2f})"
        )

    elif state == State.LAP_CHECK:
        # Slow down briefly while lap count is being verified
        speed = SPEED_DEFAULT_FORWARD // 2
        steer = 0
        action = "FORWARD"
        logger.debug("[DRIVE] LAP_CHECK → slowing down")

    elif state == State.FINAL_APPROACH:
        speed = SPEED_DEFAULT_FORWARD // 2
        steer = 0
        action = "FORWARD"
        logger.debug("[DRIVE] FINAL_APPROACH → slow forward")

    elif state == State.PARK:
        # The real PARK behaviour is the staged parallel-park maneuver in
        # planning/parking_planner.py, driven directly by runtime._resolve_parking
        # (it needs per-tick dt + IMU heading + side ToF, which this pure
        # state->command map deliberately doesn't take). drive_command is only
        # reached for PARK if something bypasses the runtime path, so the safe
        # fallback here is a full STOP -- never an open-loop reverse.
        speed = SPEED_STOP
        steer = 0
        action = "STOP"
        logger.debug("[DRIVE] PARK → STOP (maneuver handled by runtime)")

    elif state in (State.STOP, State.ERROR, State.INIT, State.WAIT_FOR_START):
        speed = SPEED_STOP
        steer = 0
        action = "STOP"
        logger.debug(f"[DRIVE] {state.name} → STOP")

    else:
        logger.warning(f"[DRIVE] Unhandled state: {state.name} — defaulting to STOP")
        speed = SPEED_STOP
        steer = 0
        action = "STOP"

    return speed, steer, action
