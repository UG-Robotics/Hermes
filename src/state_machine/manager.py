try:
    from state_machine.states import State
    from state_machine.transitions import TransitionManager
    from utils.logger import get_logger
    from state_machine.priorities import Priority
    from state_machine.robot_context import RobotContext
except ModuleNotFoundError:
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.states import State
    from state_machine.transitions import TransitionManager
    from utils.logger import get_logger
    from state_machine.priorities import Priority
    from state_machine.robot_context import RobotContext

logger = get_logger(__name__)


class StateMachine:

    def __init__(self, camera_ready: bool = True, serial_ready: bool = True):
        self.context = RobotContext()
        self.current_state = State.INIT

        self.transition_manager = TransitionManager()

        logger.info(f"INITIAL STATE: {self.current_state.name}")

        # The camera and serial link are constructed by Runtime BEFORE this
        # StateMachine is built (see runtime.py), so by the time we get here
        # their readiness is already known - we gate/report on it here rather
        # than re-initializing anything. Only the camera (needed the instant
        # START_BUTTON_PRESSED fires - must be ready for the whole of
        # WAIT_FOR_START) and the serial link (needed to move at all) are
        # hard requirements to leave INIT.
        if not camera_ready:
            logger.warning("INIT: camera NOT ready. Staying in INIT.")
        if not serial_ready:
            logger.warning("INIT: serial link NOT ready. Staying in INIT.")

        if camera_ready and serial_ready:
            self.current_state = State.WAIT_FOR_START
            logger.info("TRANSITION: INIT -> WAIT_FOR_START (camera armed, serial link up)")
        else:
            logger.error("INIT: startup checks failed, remaining in INIT until resolved.")

    def handle_event(self, event):
        logger.info(f"EVENT RECEIVED: {event.type.name}")
        logger.info(f"EVENT PRIORITY: {event.priority.name}")

        # Critical events override all transitions
        if event.priority == Priority.CRITICAL:
            logger.error("CRITICAL EVENT DETECTED")
            self.current_state = State.ERROR
            return

        # Priority-ordered event queue enforcement:
        # If the incoming event is lower priority than a
        # hypothetical concurrent higher-priority event,
        # the TransitionManager handles precedence via the
        # transition table. True multi-event priority sorting
        # belongs in the event queue (future: event_queue.py).

        new_state = self.transition_manager.get_next_state(
            self.current_state,
            event.type
        )

        self.current_state = new_state
        self.context.update_state(self.current_state.name)

        logger.info(f"CURRENT STATE: {self.current_state.name}")

    def reset(self, camera_ready: bool = True, serial_ready: bool = True):
        """Manual reset from ERROR back to INIT. Requires human intervention.

        camera_ready/serial_ready let the caller pass in current hardware
        status (a camera that died mid-run shouldn't silently be assumed
        fine again just because a human hit reset)."""
        if self.current_state == State.ERROR:
            logger.warning("Manual reset triggered. Returning to INIT.")
            self.context.reset()
            self.current_state = State.INIT
            logger.info("TRANSITION: ERROR -> INIT")
            if camera_ready and serial_ready:
                self.current_state = State.WAIT_FOR_START
                logger.info("TRANSITION: INIT -> WAIT_FOR_START")
            else:
                logger.error("INIT: startup checks still failing, remaining in INIT.")
        else:
            logger.warning(
                f"Reset ignored: not in ERROR state "
                f"(current: {self.current_state.name})"
            )