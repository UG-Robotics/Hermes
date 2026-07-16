from state_machine.states import State
from state_machine.robot_context import RobotContext
from config.robot_config import (
    SPEED_DEFAULT_FORWARD,
    SPEED_DEFAULT_BACKWARD,
    SPEED_STOP,
    STEER_CENTER_DEGREE,
)
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
        speed = SPEED_DEFAULT_FORWARD
        # This is only the intent handed to the IMU heading-hold controller
        # (control/steering_control.py) in runtime.py, which locks "straight
        # from here" and does the actual closed-loop correction, not the
        # final steer value sent to the ESP32.
        steer = 0
        action = "FORWARD"
        logger.debug(f"[DRIVE] FOLLOW_TRACK -> speed={speed}, steer intent={steer}")

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
        speed = SPEED_DEFAULT_FORWARD
        action = "FORWARD"
        logger.debug(f"[DRIVE] AVOID_OBSTACLE -> pillar={context.last_pillar_color}, steer intent={steer}")

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
        # Placeholder: parking maneuver logic goes here
        speed = SPEED_DEFAULT_BACKWARD
        steer = 0
        action = "BACKWARD"
        logger.debug("[DRIVE] PARK → reversing into spot")

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