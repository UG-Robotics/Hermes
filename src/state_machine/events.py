from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, Any
import time

try:
    from state_machine.priorities import Priority
except ModuleNotFoundError:
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.priorities import Priority


# Event priority ranking (per competition spec):
# CRITICAL_FAILURE > PILLAR_DETECTED > FINISH_ZONE_DETECTED > LAP_MARKER_DETECTED
# This is enforced at the Priority level on each Event instance.
# Multi-event sorting within a single tick is handled by the event queue.

class EventType(Enum):
    # Competition start signal
    START_BUTTON_PRESSED = auto()

    # Obstacle/pillar detection and clearance
    PILLAR_DETECTED_RED = auto()
    PILLAR_DETECTED_GREEN = auto()
    OBSTACLE_CLEARED = auto()

    # Lap counting and completion
    LAP_MARKER_DETECTED = auto()
    THREE_LAPS_COMPLETE = auto()

    # Finish/parking zone detection and completion
    FINISH_ZONE_DETECTED = auto()
    PARKING_ZONE_DETECTED = auto()
    PARKING_COMPLETE = auto()

    # Failure and timeout signals
    FAILURE_DETECTED = auto()
    TIMEOUT = auto()


# Canonical priority mapping per spec.
# Use this when constructing Events to ensure consistent priority assignment.
EVENT_PRIORITY_MAP = {
    EventType.FAILURE_DETECTED:      Priority.CRITICAL,
    EventType.PILLAR_DETECTED_RED:   Priority.HIGH,
    EventType.PILLAR_DETECTED_GREEN: Priority.HIGH,
    EventType.OBSTACLE_CLEARED:      Priority.HIGH,
    EventType.THREE_LAPS_COMPLETE:   Priority.HIGH,
    EventType.PARKING_COMPLETE:      Priority.HIGH,
    EventType.FINISH_ZONE_DETECTED:  Priority.MEDIUM,
    EventType.PARKING_ZONE_DETECTED: Priority.MEDIUM,
    EventType.LAP_MARKER_DETECTED:   Priority.MEDIUM,
    EventType.START_BUTTON_PRESSED:  Priority.MEDIUM,
    EventType.TIMEOUT:               Priority.LOW,
}


def make_event(event_type: EventType, metadata: Dict = None) -> "Event":
    """
    Factory function for creating Events with canonical priority.
    Prefer this over constructing Event() directly to ensure
    spec-compliant priority assignment.
    """
    return Event(
        type=event_type,
        priority=EVENT_PRIORITY_MAP[event_type],
        metadata=metadata or {}
    )


@dataclass
class Event:
    type: EventType
    priority: Priority
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)