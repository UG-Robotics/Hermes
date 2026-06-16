# from state_machine.events import Event, EventType
# from state_machine.priorities import Priority


# def generate_mock_events():

#     return [

#         Event(
#             type=EventType.START_BUTTON_PRESSED,
#             priority=Priority.MEDIUM
#         ),

#         Event(
#             type=EventType.PILLAR_DETECTED_RED,
#             priority=Priority.HIGH,
#             metadata={
#                 "distance": 0.42
#             }
#         ),

#         Event(
#             type=EventType.OBSTACLE_CLEARED,
#             priority=Priority.HIGH
#         ),

#         Event(
#             type=EventType.LAP_MARKER_DETECTED,
#             priority=Priority.MEDIUM
#         ),

#         Event(
#             type=EventType.THREE_LAPS_COMPLETE,
#             priority=Priority.HIGH
#         ),

#         Event(
#             type=EventType.PARKING_ZONE_DETECTED,
#             priority=Priority.MEDIUM
#         ),

#         Event(
#             type=EventType.PARKING_COMPLETE,
#             priority=Priority.HIGH
#         )
#     ]

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
    via make_event(). Update this sequence as new scenarios are needed.
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