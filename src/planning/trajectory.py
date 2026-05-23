from dataclasses import dataclass


@dataclass
class MotionTarget:
	"""Target motion command produced by planners."""

	speed: float
	steering: float
	reason: str = ""
