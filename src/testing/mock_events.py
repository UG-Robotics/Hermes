try:
    from state_machine.events import make_event, EventType
except ModuleNotFoundError:
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.events import make_event, EventType


def generate_mock_events():
    """
    Simulates a full competition run using canonical event priorities
    via make_event(). This sequence may be updated as new scenarios are needed.
    """
    return [
        make_event(EventType.START_BUTTON_PRESSED),

        make_event(EventType.PILLAR_DETECTED_RED, metadata={"distance": 0.42}),

        make_event(EventType.OBSTACLE_CLEARED),

        make_event(EventType.LAP_MARKER_DETECTED),

        make_event(EventType.THREE_LAPS_COMPLETE),

        make_event(EventType.PARKING_ZONE_DETECTED),

        make_event(EventType.PARKING_COMPLETE),
    ]