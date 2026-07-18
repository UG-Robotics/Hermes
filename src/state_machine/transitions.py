try:
    from state_machine.states import State
    from state_machine.events import EventType
    from utils.logger import get_logger
except ModuleNotFoundError:
    # Allow running this file directly by adding src to sys.path
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.states import State
    from state_machine.events import EventType
    from utils.logger import get_logger
    
logger = get_logger(__name__)
    

class TransitionManager:
    
    def __init__(self):

        self.transitions = {

            # Wait for official start signal
            (State.WAIT_FOR_START,
             EventType.START_BUTTON_PRESSED): State.FOLLOW_TRACK,

            # Follow track transitions
            (State.FOLLOW_TRACK,
             EventType.PILLAR_DETECTED_RED): State.AVOID_OBSTACLE,

            (State.FOLLOW_TRACK,
             EventType.PILLAR_DETECTED_GREEN): State.AVOID_OBSTACLE,

            (State.FOLLOW_TRACK,
             EventType.LAP_MARKER_DETECTED): State.LAP_CHECK,

            (State.FOLLOW_TRACK,
             EventType.FINISH_ZONE_DETECTED): State.PARK,

            # Avoid obstacle transitions
            (State.AVOID_OBSTACLE,
             EventType.OBSTACLE_CLEARED): State.FOLLOW_TRACK,

            # Lap check transitions
            (State.LAP_CHECK,
             EventType.LAP_MARKER_DETECTED): State.FOLLOW_TRACK,
            (State.LAP_CHECK,
             EventType.THREE_LAPS_COMPLETE): State.FINAL_APPROACH,

            # NOTE: runtime.py raises THREE_LAPS_COMPLETE from inside
            # _post_event_effects while handling the LAP_CHECK-exiting
            # LAP_MARKER_DETECTED event, but state_machine/event_queue.py's
            # EventQueue.drain() snapshots the whole queue before the
            # for-loop starts (see runtime.py's _drain_events) -- so that
            # freshly-pushed THREE_LAPS_COMPLETE is never in THIS tick's
            # batch. By the time it's actually processed (next tick), the
            # LAP_MARKER_DETECTED transition above has already fired and
            # current_state is back to FOLLOW_TRACK, not LAP_CHECK. This is
            # the mapping that's actually reachable; the LAP_CHECK one above
            # is kept for robustness (e.g. a manually-injected event) but
            # isn't the one the real perception path hits.
            (State.FOLLOW_TRACK,
             EventType.THREE_LAPS_COMPLETE): State.FINAL_APPROACH,

            # Final approach transitions
            (State.FINAL_APPROACH,
             EventType.PARKING_ZONE_DETECTED): State.PARK,
            (State.FINAL_APPROACH,
             EventType.FINISH_ZONE_DETECTED): State.PARK,

            # Parking transitions
            (State.PARK,
             EventType.PARKING_COMPLETE): State.STOP,
        }

    def get_next_state(self, current_state, event_type):

        # Critical failures always go to ERROR
        if event_type == EventType.FAILURE_DETECTED:
            logger.error(
                "CRITICAL FAILURE DETECTED"
            )
            return State.ERROR

        key = (current_state, event_type)

        if key in self.transitions:
            next_state = self.transitions[key]

            logger.info(
                f"TRANSITION: {current_state.name} -> {next_state.name}"
            )

            return next_state

        logger.warning(
            f"INVALID TRANSITION: "
            f"{current_state.name} + {event_type.name}"
        )

        return current_state
