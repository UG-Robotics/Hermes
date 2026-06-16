try:
    from state_machine.states import State
    from state_machine.transitions import TransitionManager
    from utils.logger import get_logger
    from state_machine.priorities import Priority
except ModuleNotFoundError:
    # Allow running this file directly by adding src to sys.path
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.states import State
    from state_machine.transitions import TransitionManager
    from utils.logger import get_logger
    from state_machine.priorities import Priority

logger = get_logger(__name__)


class StateMachine:

    def __init__(self):

        self.current_state = State.INIT

        self.transition_manager = TransitionManager()

        logger.info(
            f"INITIAL STATE: {self.current_state.name}"
        )
        # Initialization complete; wait for official start signal
        self.current_state = State.WAIT_FOR_START
        logger.info(
            "TRANSITION: INIT -> WAIT_FOR_START"
        )

    def handle_event(self, event):

        logger.info(
            f"EVENT RECEIVED: {event.type.name}"
        )

        logger.info(
            f"EVENT PRIORITY: {event.priority.name}"
        )

        # Critical events override all transitions
        if event.priority == Priority.CRITICAL:

            logger.error(
                "CRITICAL EVENT DETECTED"
            )

            self.current_state = State.ERROR

            return

        # Normal transitions based on current state and event
        new_state = self.transition_manager.get_next_state(
            self.current_state,
            event.type
        )

        self.current_state = new_state

        logger.info(
            f"CURRENT STATE: {self.current_state.name}"
        )
