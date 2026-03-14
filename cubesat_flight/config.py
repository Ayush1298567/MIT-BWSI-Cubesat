# config.py — All mission parameters for MuraltZ CubeSat flight software

MISSION_NAME = "MuraltZ"
CUBESAT_ID = "MURALTZ-01"

# === STATE TIMING ===
BOOT_TIMEOUT = 30               # sec — max time allowed for boot self-test
IMAGING_WINDOW_SEC = 75         # sec — max duration of one imaging pass
IDLE_DURATION_SEC = 30          # sec — queue build / cleanup after imaging
DOWNLINK_WINDOW_SEC = 60        # sec — max time allocated for one downlink pass
PASS_CYCLE_DELAY_SEC = 10       # sec — pause between downlink end and next imaging

# === CAMERA ===
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
JPEG_QUALITY = 70               # Primary capture quality
JPEG_MAX_SIZE_BYTES = 40000     # If file > this at JPEG_QUALITY, recompress at JPEG_QUALITY-10
CAPTURE_INTERVAL_SEC = 3.0      # Seconds between capture attempts in imaging loop
MAX_IMAGES_PER_PASS = 20        # Hard cap on captures per pass

# Camera runs in AUTO-EXPOSURE mode by default. We do NOT manually calibrate
# exposure because the flashlight creates uneven brightness across the surface.
# Auto-exposure continuously adapts. Real auto-chosen values are read from
# get_metadata() after each capture and logged in the _meta.json sidecar.
CAMERA_USE_AUTO_EXPOSURE = True

# === CAMERA CALIBRATION (MEASURED — fill in before demo) ===
CAMERA_HEIGHT_CM = 0.0          # Measured lens-to-surface distance in cm
CAMERA_FOV_H_DEG = 62.2         # Horizontal field of view, Pi Camera Module 3
CAMERA_FOV_V_DEG = 48.8         # Vertical field of view, Pi Camera Module 3

# === IMU THRESHOLDS ===
# Loosened for handheld operation. A real satellite at 100 km would be tighter.
ANGULAR_RATE_THRESHOLD = 1.0    # rad/s — max gyro magnitude to be considered stable
NADIR_TOLERANCE_DEG = 45        # degrees — nadir lock engages below this angle
# Hysteresis: once nadir is locked (angle < NADIR_TOLERANCE_DEG), the lock
# is held until the angle exceeds NADIR_EXIT_DEG. This prevents rapid
# capture-skip toggling when hovering near the threshold.
NADIR_EXIT_DEG = 55             # degrees — nadir lock releases above this angle

# === QUALITY GATES ===
# BLUR_THRESHOLD MUST be calibrated against the real surface before demo day.
# Procedure: take 10 sharp + 10 blurry photos of the actual surface, compute
# cv2.Laplacian(gray, cv2.CV_64F).var() on all 20, then set threshold to the
# midpoint between the lowest sharp variance and highest blurry variance.
# Sand/regolith has low texture variance even when in focus — the default of 50
# is a conservative guess and WILL need adjustment.
BLUR_THRESHOLD = 50.0           # CALIBRATE THIS before demo. Default is a guess.
EXPOSURE_MIN = 15               # Pixel mean below this → underexposed rejection
EXPOSURE_MAX = 240              # Pixel mean above this → overexposed rejection
QUALITY_WEIGHT_BLUR = 0.3
QUALITY_WEIGHT_EXPOSURE = 0.25
QUALITY_WEIGHT_MOTION = 0.2
QUALITY_WEIGHT_NOVELTY = 0.25

# === COVERAGE GRID ===
GRID_ROWS = 8
GRID_COLS = 8
# Grid cell is set by operator terminal input ("cell R C"), NOT by elapsed time
# or any trajectory map. All captures are tagged with current_grid_cell until
# the operator updates it. This prevents desync from timing drift.
DEFAULT_GRID_CELL = (0, 0)

# === DOWNLINK ===
GROUND_STATION_IP = "192.168.1.100"
DATA_PORT = 5000
COMMAND_PORT = 5001
THROTTLE_BYTES_PER_SEC = 1200   # 9600 bps / 8 = UHF flight link equivalent
ACK_TIMEOUT_SEC = 5
MAX_RETRIES_PER_IMAGE = 3

# Data budget is computed at runtime — not a separate constant.
# DATA_BUDGET = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC = 72,000 bytes
# At ~28 KB per image, that is ~2 images per downlink window.
# This is the real bottleneck — the priority queue decides which 2 get sent.

# Max consecutive GCS connection failures before downlink is suspended.
# Downlink resumes when a "retry_downlink" command arrives from GCS.
MAX_GCS_CONNECT_FAILURES = 3

# === PRIORITY TIERS ===
# P2 images age to P3 after this many passes. With a 3-pass demo this fires
# during pass 3's IDLE for pass 1's P2 images.
P2_AGING_PASSES = 2

# === STORAGE PATHS ===
IMAGE_DIR = "/home/cubesat/cubesat_flight/data/images"
TELEMETRY_DIR = "/home/cubesat/cubesat_flight/data/telemetry"
LOG_DIR = "/home/cubesat/cubesat_flight/data/logs"
QUEUE_FILE = "/home/cubesat/cubesat_flight/data/queue.json"
IMAGE_INDEX_FILE = "/home/cubesat/cubesat_flight/data/image_index.json"
RECOVERY_FILE = "/home/cubesat/cubesat_flight/data/recovery.json"
STORAGE_WARNING_PCT = 80        # Start cleanup of P3 images above this
STORAGE_CRITICAL_PCT = 98       # Stop imaging above this

# === THERMAL ===
CPU_TEMP_WARNING_C = 70         # Double capture interval above this
CPU_TEMP_CRITICAL_C = 80        # Enter safe mode above this
THERMAL_CHECK_INTERVAL_SEC = 10

# === WATCHDOG ===
WATCHDOG_TIMEOUT_SEC = 30       # No pet() within this window → restart
WATCHDOG_CHECK_INTERVAL_SEC = 5

# === MEASURED POWER (fill in from USB power meter readings) ===
MEASURED_IDLE_POWER_W = 0.0
MEASURED_IMAGING_POWER_W = 0.0
MEASURED_PROCESSING_POWER_W = 0.0
MEASURED_DOWNLINK_POWER_W = 0.0

# === DEMO SCALE ===
DEMO_GSD_MM_PER_PX = 0.12      # Ground sample distance in demo (mm/px)
FLIGHT_GSD_M_PER_PX = 29.5     # Ground sample distance in actual flight (m/px)
SCALE_FACTOR = 250_000          # Demo to flight scale ratio
