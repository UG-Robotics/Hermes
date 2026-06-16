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
        steer = 0  # straight — replace with PID/lane output later
        action = "FORWARD"
        logger.debug(f"[DRIVE] FOLLOW_TRACK → speed={speed}, steer={steer}")

    elif state == State.AVOID_OBSTACLE:
        # Placeholder: steer based on last seen pillar color
        # Red pillar → go left (negative steer), Green → go right
        if context.last_pillar_color == "RED":
            steer = -30
        elif context.last_pillar_color == "GREEN":
            steer = 30
        else:
            steer = 0
        speed = SPEED_DEFAULT_FORWARD
        action = "FORWARD"
        logger.debug(f"[DRIVE] AVOID_OBSTACLE → pillar={context.last_pillar_color}, steer={steer}")

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