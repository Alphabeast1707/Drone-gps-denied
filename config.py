"""
Configuration for GPS-Denied Drone Vision System
Raspberry Pi 4 Model B (8GB)

Optimized for high-FPS AprilTag detection + precision landing.
"""

# ──────────────────────────────────────────────
# Camera settings
# ──────────────────────────────────────────────
CAMERA_INDEX = 0              # 0 for CSI / USB camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 90               # Target FPS (CSI camera supports up to 90)

# ──────────────────────────────────────────────
# Camera intrinsic parameters (approximate)
# You should calibrate your own camera for best results.
# These are rough defaults for Raspberry Pi Camera v2 at 640x480.
# ──────────────────────────────────────────────
CAMERA_FX = 600.0             # Focal length X (pixels)
CAMERA_FY = 600.0             # Focal length Y (pixels)
CAMERA_CX = 320.0             # Principal point X (pixels)
CAMERA_CY = 240.0             # Principal point Y (pixels)
CAMERA_PARAMS = [CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY]

# ──────────────────────────────────────────────
# AprilTag detector settings (high-speed config)
# ──────────────────────────────────────────────
TAG_FAMILY = "tag36h11"
TAG_NTHREADS = 4              # Use all 4 Cortex-A72 cores
TAG_QUAD_DECIMATE = 3.0       # Downsample factor (3-4 for speed)
TAG_QUAD_SIGMA = 0.0          # No Gaussian blur
TAG_REFINE_EDGES = False      # Disable for speed (True for accuracy)
TAG_DECODE_SHARPENING = 0.25

# ──────────────────────────────────────────────
# Tag sizes (meters) — physical size of printed tags
# ──────────────────────────────────────────────
CORNER_TAG_SIZE = 0.20        # 20 cm corner tags (Tag IDs 1-4)
CENTER_TAG_SIZE = 0.10        # 10 cm center tag  (Tag ID 5)

# ──────────────────────────────────────────────
# Tag IDs for multi-tag landing pad
# ──────────────────────────────────────────────
CORNER_TAG_IDS = [1, 2, 3, 4]   # Corner guidance tags
CENTER_TAG_ID = 5                # Precision landing tag

# ──────────────────────────────────────────────
# Landing controller thresholds
# ──────────────────────────────────────────────
# Pixel offset threshold for centering (pixels)
CENTER_THRESHOLD_PX = 20

# Distance thresholds (meters)
DESCEND_ALTITUDE = 1.0        # Start descending below this
SLOW_DESCENT_ALTITUDE = 0.5   # Slow descent below this
LAND_ALTITUDE = 0.2           # Trigger landing below this

# ──────────────────────────────────────────────
# ROI tracking settings
# ──────────────────────────────────────────────
ROI_ENABLED = True
ROI_MARGIN = 100              # Pixels around last known tag position
ROI_LOST_FRAMES = 10          # Frames without detection before full scan

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
