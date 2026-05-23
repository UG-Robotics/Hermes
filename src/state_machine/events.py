from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, Any
import time

try:
    from state_machine.priorities import Priority
except ModuleNotFoundError:
    # Allow running this file directly by adding src to sys.path
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.priorities import Priority


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
    


@dataclass
class Event:
    # Event container with metadata and timestamp
    type: EventType
    priority: Priority
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
