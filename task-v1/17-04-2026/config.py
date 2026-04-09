import numpy as np

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOCAL_LENGTH_PX = 480.0
MARKER_SIZE = 0.15

CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH_PX, 0, IMAGE_WIDTH/2],
    [0, FOCAL_LENGTH_PX, IMAGE_HEIGHT/2],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.03, -0.01, 0, 0, 0], dtype=np.float32)

PITCH_ANGLE_RAD = np.deg2rad(15)
UNDERWATER_REFRACTION = 1.33

MAX_LATERAL = 0.35
MAX_YAW = 0.25
MAX_VERTICAL = 0.18

DIST_ALIGN = 3.0
DIST_DESCENDING = 0.6
FINAL_ALIGN_DIST = 0.25
DIST_DOCKED = 0.15

PRECISION_THRESH = 0.03
HOVER_TIME = 1.2
FAILSAFE_LOST = 4.0

SERIAL_PORT = '/dev/ttyACM0'  # Jetson üzerindeki Pixhawk portu
BAUD_RATE = 115200