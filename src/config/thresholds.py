# HSV ranges for pillar colors. Values are tuned for typical indoor lighting.
PILLAR_HSV_RANGES = {
	"red": [
		((0, 120, 70), (10, 255, 255)),
		((170, 120, 70), (179, 255, 255)),
	],
	"green": [
		((35, 80, 60), (85, 255, 255)),
	],
}

# Minimum blob area to qualify as a pillar detection (in pixels).
PILLAR_MIN_AREA = 500
