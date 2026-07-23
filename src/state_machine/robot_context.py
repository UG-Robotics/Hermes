from dataclasses import dataclass, field
from datetime import datetime
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class RobotContext:

    lap_count: int = 0
    target_laps: int = 3

    current_state: str = "INIT"
    previous_state: str | None = None

    last_pillar_color: str | None = None

    # Set by the pillar-avoidance pipeline (perception/pillar_detection.py via
    # runtime.py) whenever a pillar is detected/latched. steer is the P-control
    # avoidance angle in the same -90..90 units as everything else; distance_mm
    # is the monocular distance estimate to the pillar (None if unknown).
    pillar_steer_angle: int = 0
    pillar_distance_mm: float | None = None

    # IMU heading-hold target, in degrees, set by control/steering_control.py.
    # None means "no lock yet" (e.g. before the IMU has produced a reading).
    target_heading_deg: float | None = None

    obstacle_active: bool = False

    # Latest side ToF readings, refreshed every tick by
    # runtime.py's _refresh_tof_context() right after telemetry ingest.
    # Always a float (never None) -- 2000.0 (TOF_MAX_VALID_MM) means "no
    # wall in range", matching hardware/tof.py's ToFArray.OUT_OF_RANGE_MM
    # default, so consumers never have to null-check it.
    tof_left_mm: float = 2000.0
    tof_right_mm: float = 2000.0

    # Latest ToF-based corridor-centre offset (see planning/wall_centering.py).
    # centering_offset_mm is the signed lateral offset in mm (+ = displaced
    # toward the right wall), None when no wall is in range. centering_mode is
    # "BOTH" | "LEFT_WALL" | "RIGHT_WALL" | "NONE" -- which regime produced it.
    centering_offset_mm: float | None = None
    centering_mode: str = "NONE"

    # Corner-marker bookkeeping (see perception/corner_detection.py +
    # runtime.py's _post_event_effects). corners_passed counts every
    # completed corner traversal since the run started; a lap completes
    # every CORNERS_PER_LAP of them. race_direction is latched from the
    # colour of the first corner marker ever seen ("CLOCKWISE" |
    # "COUNTER_CLOCKWISE"), None until then.
    corners_passed: int = 0
    race_direction: str | None = None

    # Which challenge this run is (auto-detected -- WRO FE allows only one
    # start button, so the car can't be told). None until decided;
    # "OBSTACLE" the instant a red/green pillar is seen; "OPEN" if a full lap
    # is completed without any pillar. See config/thresholds.py's challenge
    # separation notes and runtime.py's _post_event_effects.
    challenge_mode: str | None = None
    # Latches True the first time any pillar is detected this run -- the
    # signal challenge_mode == "OBSTACLE" is derived from.
    pillar_ever_seen: bool = False

    parking_detected: bool = False
    parking_complete: bool = False

    speed: int = 0
    steering: int = 0

    heading: float = 0.0

    estimated_track_position: str | None = None

    error_flag: bool = False
    error_message: str = ""

    run_start_time: datetime = field(
        default_factory=datetime.now
    )

    def increment_lap(self):

        self.lap_count += 1

        logger.info(
            f"Lap completed. "
            f"Current lap count = {self.lap_count}"
        )

    def update_state(self, new_state: str):

        old_state = self.current_state

        self.previous_state = old_state

        self.current_state = new_state

        logger.info(
            f"State transition: "
            f"{old_state} -> {new_state}"
        )

    def set_error(self, message: str):

        self.error_flag = True

        self.error_message = message

        logger.error(
            f"Robot Error: {message}"
        )

    def clear_error(self):

        if self.error_flag:

            logger.info(
                "Robot error cleared."
            )

        self.error_flag = False
        self.error_message = ""

    def reset(self):

        logger.warning(
            "RobotContext reset."
        )

        self.lap_count = 0
        self.target_laps = 3

        self.current_state = "INIT"
        self.previous_state = None

        self.last_pillar_color = None
        self.pillar_steer_angle = 0
        self.pillar_distance_mm = None
        self.target_heading_deg = None

        self.obstacle_active = False

        self.tof_left_mm = 2000.0
        self.tof_right_mm = 2000.0
        self.centering_offset_mm = None
        self.centering_mode = "NONE"

        self.corners_passed = 0
        self.race_direction = None

        self.challenge_mode = None
        self.pillar_ever_seen = False

        self.parking_detected = False
        self.parking_complete = False

        self.speed = 0
        self.steering = 0

        self.heading = 0.0

        self.estimated_track_position = None

        self.error_flag = False
        self.error_message = ""

        self.run_start_time = datetime.now()

    def __str__(self):

        return (
            f"RobotContext("
            f"lap_count={self.lap_count}, "
            f"state={self.current_state}, "
            f"previous={self.previous_state}, "
            f"pillar={self.last_pillar_color}, "
            f"obstacle={self.obstacle_active}, "
            f"parking={self.parking_complete}, "
            f"error={self.error_flag}"
            f")"
        )