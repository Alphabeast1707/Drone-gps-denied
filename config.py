"""
Configuration for GPS-Denied Drone Vision System
Raspberry Pi 4 Model B (8GB)

Optimized for high-FPS AprilTag detection + precision landing.

Base Station Dimensions (from engineering drawing):
  Total pad:       ~688.91 mm  (150 + 388.91 + 150)
  Corner tags:     150 × 150 mm  (4 tags at corners)
  Center tag:      ~100 × 100 mm (1 tag at center)
  Inner circle:    R = 163.48 mm
  Outer circle:    R = 275 mm
"""

# ──────────────────────────────────────────────
# Camera settings
# ──────────────────────────────────────────────
CAMERA_INDEX = 0              # 0 for USB webcam (Microsoft LifeCam HD-5000)
CAMERA_BACKEND = "V4L2"       # Force V4L2 backend (not GStreamer)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30               # Max 30 fps for LifeCam HD-5000 at 640x480
CAMERA_FOURCC = "MJPG"        # Use MJPG for best FPS (YUYV is slower on USB)

# ──────────────────────────────────────────────
# Camera intrinsic parameters (CALIBRATED)
# Calibrated on 2026-03-13 with 20 checkerboard images
# Reprojection error: 0.1236 px
# Camera: Microsoft LifeCam HD-5000 at 640x480
# ──────────────────────────────────────────────
CAMERA_FX = 690.11            # Focal length X (pixels)
CAMERA_FY = 691.40            # Focal length Y (pixels)
CAMERA_CX = 303.24            # Principal point X (pixels)
CAMERA_CY = 187.94            # Principal point Y (pixels)
CAMERA_PARAMS = [CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY]

# Distortion coefficients [k1, k2, p1, p2, k3]
CAMERA_DIST_COEFFS = [-0.000930, 0.421920, -0.024044, -0.021204, -1.327173]

# ──────────────────────────────────────────────
# AprilTag detector settings (high-speed config)
#
# NOTE: decimate=2.0 (not 3) because corner tags are only 15cm.
#       Smaller tags need more resolution to be reliably detected.
#       At 3m+ altitude, 15cm tags at decimate=3 may be missed.
# ──────────────────────────────────────────────
TAG_FAMILY = "tag36h11"
TAG_NTHREADS = 4              # Use all 4 Cortex-A72 cores
TAG_QUAD_DECIMATE = 2.0       # 2.0 for 15cm tags (use 3.0 only if FPS is critical)
TAG_QUAD_SIGMA = 0.0          # No Gaussian blur
TAG_REFINE_EDGES = True       # Enable for accuracy (15cm tags need it)
TAG_DECODE_SHARPENING = 0.25

# ──────────────────────────────────────────────
# Base station physical dimensions (meters)
# From engineering drawing
# ──────────────────────────────────────────────
PAD_TOTAL_WIDTH = 0.68891     # ~688.91 mm total pad width
PAD_INNER_WIDTH = 0.38891     # 388.91 mm between corner tags
PAD_INNER_CIRCLE_R = 0.16348  # Inner circle radius (163.48 mm)
PAD_OUTER_CIRCLE_R = 0.275    # Outer circle radius (275 mm)

# ──────────────────────────────────────────────
# Tag sizes (meters) — physical size of printed tags
# ──────────────────────────────────────────────
CORNER_TAG_SIZE = 0.15        # 15 cm (150 mm) corner tags
CENTER_TAG_SIZE = 0.10        # 10 cm (100 mm) center tag

# ──────────────────────────────────────────────
# Tag IDs for multi-tag landing pad
# ──────────────────────────────────────────────
CORNER_TAG_IDS = [1, 2, 3, 4]   # Corner guidance tags
CENTER_TAG_ID = 5                # Precision landing tag

# Corner tag positions relative to pad center (meters)
# Used to estimate pad center even if only some corners are visible
#   Tag1 = top-left, Tag2 = top-right
#   Tag3 = bottom-left, Tag4 = bottom-right
CORNER_OFFSETS = {
    1: (-0.19445, -0.19445),  # top-left     (half of 388.91mm)
    2: ( 0.19445, -0.19445),  # top-right
    3: (-0.19445,  0.19445),  # bottom-left
    4: ( 0.19445,  0.19445),  # bottom-right
}

# ──────────────────────────────────────────────
# Landing controller thresholds
# ──────────────────────────────────────────────
# Pixel offset threshold for centering (pixels)
CENTER_THRESHOLD_PX = 15      # Tighter threshold for 69cm pad

# Distance thresholds (meters) — tuned for this pad size
APPROACH_ALTITUDE = 3.0       # Above this: use corner tags to approach
DESCEND_ALTITUDE = 1.5        # Switch to center tag + descend
SLOW_DESCENT_ALTITUDE = 0.5   # Slow descent for precision
LAND_ALTITUDE = 0.20          # Trigger landing (20cm above pad)

# Horizontal correction gains (proportional control)
# Higher = more aggressive correction
HORIZONTAL_GAIN = 0.5         # m/s per meter of offset

# ──────────────────────────────────────────────
# ROI tracking settings
# ──────────────────────────────────────────────
ROI_ENABLED = True
ROI_MARGIN = 180              # Pixels — large enough for full pad at low altitude
ROI_LOST_FRAMES = 15          # Frames without detection before full scan

# ──────────────────────────────────────────────
# Telemetry logging
# ──────────────────────────────────────────────
LOG_FILE = "landing_log.csv"
LOG_ENABLED = True

# ──────────────────────────────────────────────
# Visual odometry settings
# ──────────────────────────────────────────────
VO_MAX_FEATURES = 500         # ORB feature count
VO_MIN_MATCHES = 20           # Minimum matches for motion estimation
