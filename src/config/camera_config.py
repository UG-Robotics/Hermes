# camera + vision pipeline constants
#
# All HSV bounds are OpenCV convention: H in [0,179], S/V in [0,255].
# These defaults are reasonable starting points for a Pi Camera V2 under
# indoor competition lighting — they WILL need a quick on-mat calibration
# pass (see testing/vision/pillar_vision_test.py) before competition day.

# ---------------------------------------------------------------- resolution
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# ------------------------------------------------------------- colour ranges
# Red wraps around the 0/180 hue boundary, so it needs two ranges OR'd
# together. Green does not wrap and needs only one.
HSV_RED_LOW1 = (0, 120, 70)
HSV_RED_HIGH1 = (10, 255, 255)
HSV_RED_LOW2 = (170, 120, 70)
HSV_RED_HIGH2 = (179, 255, 255)

HSV_GREEN_LOW = (40, 70, 60)
HSV_GREEN_HIGH = (85, 255, 255)

# Magenta (parking zone marker) is intentionally defined but NOT used by
# pillar_detection yet — parking is a later milestone. Kept here so the hue
# windows for red/green/magenta are chosen together and don't collide.
HSV_MAGENTA_LOW = (140, 90, 90)
HSV_MAGENTA_HIGH = (169, 255, 255)

# ------------------------------------------------------------ blob filtering
MIN_PILLAR_AREA = 250          # px^2 — below this, treat as noise
PILLAR_LOST_FRAMES = 5         # consecutive empty frames before "cleared"

# ---------------------------------------------------- steering decision (P)
# We steer toward an "aim point" offset from the frame centre rather than
# straight at the pillar's centroid: RED -> aim right of centre (pass on the
# pillar's right), GREEN -> aim left of centre (pass on the pillar's left).
# error = aim_point_x - pillar_cx ; steer = clamp(kp * error)
PILLAR_LATERAL_OFFSET_PX = 70
PILLAR_STEER_KP = 0.45
PILLAR_MIN_STEER_DEG = 18       # once a pillar is confirmed, always steer at least this much
PILLAR_MAX_STEER_DEG = 60       # cap avoidance steer (leave margin under STEER_MAX)

# ------------------------------------------------- monocular distance estimate
# distance_mm ≈ (real_width_mm * focal_px) / pixel_width_px
# CAMERA_FOCAL_PX must be calibrated for the real lens/resolution in use;
# this default assumes a Pi Camera V2 at FRAME_WIDTH above and a pillar
# roughly 1 m away filling ~60px of frame width. Recalibrate on the mat.
PILLAR_REAL_DIAMETER_MM = 65.0
CAMERA_FOCAL_PX = 350.0

# Once the estimated distance drops under this, we consider ourselves
# alongside the pillar (about to pass it) rather than still approaching it.
PILLAR_CLEAR_DISTANCE_MM = 180.0
