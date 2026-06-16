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

    obstacle_active: bool = False

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

        self.current_state = "INIT"
        self.previous_state = None

        self.last_pillar_color = None

        self.obstacle_active = False

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
        
        
ctx = RobotContext()

logger.info(ctx)