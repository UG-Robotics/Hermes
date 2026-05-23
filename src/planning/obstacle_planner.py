from dataclasses import dataclass
from enum import Enum

from config.robot_config import (
	DEFAULT_SPEED,
	DEFAULT_STEERING_OFFSET,
	OBSTACLE_SPEED_SCALE,
)
from planning.trajectory import MotionTarget


class ObstacleSide(Enum):
	LEFT = "left"
	RIGHT = "right"
	UNKNOWN = "unknown"


@dataclass
class ObstaclePlan:
	side: ObstacleSide
	target: MotionTarget


def side_from_color(color: str) -> ObstacleSide:
	if not color:
		return ObstacleSide.UNKNOWN

	color = color.lower()

	if color == "red":
		return ObstacleSide.LEFT
	if color == "green":
		return ObstacleSide.RIGHT

	return ObstacleSide.UNKNOWN


def plan_for_pillar(
	color: str,
	base_speed: float = DEFAULT_SPEED,
	base_steering: float = 0.0,
	steering_offset: float = DEFAULT_STEERING_OFFSET,
	speed_scale: float = OBSTACLE_SPEED_SCALE,
) -> ObstaclePlan:
	side = side_from_color(color)

	if side == ObstacleSide.LEFT:
		steering = base_steering - steering_offset
	elif side == ObstacleSide.RIGHT:
		steering = base_steering + steering_offset
	else:
		steering = base_steering

	target = MotionTarget(
		speed=base_speed * speed_scale,
		steering=steering,
		reason=f"avoid_{side.value}",
	)

	return ObstaclePlan(side=side, target=target)
