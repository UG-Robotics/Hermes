# try:
#     from state_machine.states import State
#     from state_machine.transitions import TransitionManager
#     from utils.logger import get_logger
#     from state_machine.priorities import Priority
#     from state_machine.robot_context import RobotContext 
# except ModuleNotFoundError:
#     # Allow running this file directly by adding src to sys.path
#     import sys
#     import pathlib
#     sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
#     from state_machine.states import State
#     from state_machine.transitions import TransitionManager
#     from utils.logger import get_logger
#     from state_machine.priorities import Priority
#     from state_machine.robot_context import RobotContext

# logger = get_logger(__name__)


# class StateMachine:

#     def __init__(self):

#         class StateMachine:
#             def __init__(self):
#                 self.context = RobotContext()  # add this line
#                 self.current_state = State.INIT
#                 ...
                
#         self.current_state = State.INIT

#         self.transition_manager = TransitionManager()

#         logger.info(
#             f"INITIAL STATE: {self.current_state.name}"
#         )
#         # Initialization complete; wait for official start signal
#         self.current_state = State.WAIT_FOR_START
#         logger.info(
#             "TRANSITION: INIT -> WAIT_FOR_START"
#         )

#     def handle_event(self, event):

#         logger.info(
#             f"EVENT RECEIVED: {event.type.name}"
#         )

#         logger.info(
#             f"EVENT PRIORITY: {event.priority.name}"
#         )

#         # Critical events override all transitions
#         if event.priority == Priority.CRITICAL:

#             logger.error(
#                 "CRITICAL EVENT DETECTED"
#             )

#             self.current_state = State.ERROR

#             return

#         # Normal transitions based on current state and event
#         new_state = self.transition_manager.get_next_state(
#             self.current_state,
#             event.type
#         )

#         self.current_state = new_state

#         logger.info(
#             f"CURRENT STATE: {self.current_state.name}"
#         )


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

    def __init__(self):
        self.context = RobotContext()
        self.current_state = State.INIT

        self.transition_manager = TransitionManager()

        logger.info(f"INITIAL STATE: {self.current_state.name}")

        # --- INIT: System startup checks ---
        # TODO: initialize camera         → from perception.camera import Camera
        # TODO: initialize serial link    → from communication.serial_link import SerialLink
        # TODO: initialize PID controller → from control.pid import PIDController
        # TODO: verify sensors connected  → TOF, IMU health checks
        # When each module is ready, initialize it here and gate
        # the transition to WAIT_FOR_START on all checks passing.
        # For now, we assume all systems nominal and proceed immediately.

        self.current_state = State.WAIT_FOR_START
        logger.info("TRANSITION: INIT -> WAIT_FOR_START")

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

    def reset(self):
        """Manual reset from ERROR back to INIT. Requires human intervention."""
        if self.current_state == State.ERROR:
            logger.warning("Manual reset triggered. Returning to INIT.")
            self.context.reset()
            self.current_state = State.INIT
            logger.info("TRANSITION: ERROR -> INIT")
            # Re-run INIT checks when implemented
            self.current_state = State.WAIT_FOR_START
            logger.info("TRANSITION: INIT -> WAIT_FOR_START")
        else:
            logger.warning(
                f"Reset ignored: not in ERROR state "
                f"(current: {self.current_state.name})"
            )