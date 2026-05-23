from enum import Enum, auto


class State(Enum):
    INIT = auto()
    WAIT_FOR_START = auto()
    FOLLOW_TRACK = auto()
    AVOID_OBSTACLE = auto()
    LAP_CHECK = auto()
    FINAL_APPROACH = auto()
    PARK = auto()
    STOP = auto()
    ERROR = auto()

