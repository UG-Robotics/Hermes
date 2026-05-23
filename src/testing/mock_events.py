from state_machine.events import Event, EventType
from state_machine.priorities import Priority


def generate_mock_events():

    return [

        Event(
            type=EventType.START_BUTTON_PRESSED,
            priority=Priority.MEDIUM
        ),

        Event(
            type=EventType.PILLAR_DETECTED_RED,
            priority=Priority.HIGH,
            metadata={
                "distance": 0.42
            }
        ),

        Event(
            type=EventType.OBSTACLE_CLEARED,
            priority=Priority.HIGH
        ),

        Event(
            type=EventType.LAP_MARKER_DETECTED,
            priority=Priority.MEDIUM
        ),

        Event(
            type=EventType.THREE_LAPS_COMPLETE,
            priority=Priority.HIGH
        ),

        Event(
            type=EventType.PARKING_ZONE_DETECTED,
            priority=Priority.MEDIUM
        ),

        Event(
            type=EventType.PARKING_COMPLETE,
            priority=Priority.HIGH
        )
    ]
