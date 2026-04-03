import numpy as np
import cv2

IMAGE_WIDTH  = 1920
IMAGE_HEIGHT = 1080
FOCAL_LENGTH_PX = 1080.0
MARKER_SIZE  = 0.15

CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH_PX, 0, IMAGE_WIDTH / 2],
    [0, FOCAL_LENGTH_PX, IMAGE_HEIGHT / 2],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.03, -0.01, 0, 0, 0], dtype=np.float32)

# kamera 30 derece aşağı eğik
CAMERA_PITCH_DEG = -30.0
# suyun altında ışığın kırılması
UNDERWATER_REFRACTION = 1.33

# hız limitleri
MAX_LATERAL  = 0.35
MAX_YAW      = 0.25
MAX_VERTICAL = 0.18

DEADZONE_PX = 20 # hedef 20 piksel sapsa da merkezde say

DIST_ALIGN      = 3.0   # 3 metreden itibaren yönünü platforma çevir
DIST_APPROACH   = 1.5   # 1.5 metre kala üzerine doğru gitmeye başla
DIST_DESCENDING = 0.6   # 60 cm kalınca dikey inişe geç
DIST_DOCKED     = 0.20  # 20 cm kalınca "kondu" kabul et ve dur

FAILSAFE_LOST = 5.0 # 5 sn den sonra hedef kaybolursa dur
THRUSTER_DEADBAND = 0.10 # %10'dan az güç uygulandığında thruster hareket etmesin

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200


def build_ipm_matrix():
    pitch = np.deg2rad(CAMERA_PITCH_DEG)
    fx, fy = CAMERA_MATRIX[0,0], CAMERA_MATRIX[1,1]
    cx, cy = CAMERA_MATRIX[0,2], CAMERA_MATRIX[1,2]

    # R: Kameranın eğikliğini temsil eden rotasyon matrisi
    R = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch),  np.cos(pitch)]
    ])

    corners = np.array([
        [0,0],[IMAGE_WIDTH,0],
        [IMAGE_WIDTH,IMAGE_HEIGHT],[0,IMAGE_HEIGHT]
    ], dtype=np.float32)

    ground = []
    for u,v in corners:
        ray = np.array([(u-cx)/fx, (v-cy)/fy, 1])
        ray = R.T @ ray
        t = -1.0 / ray[2]
        ground.append([ray[0]*t, ray[1]*t])

    ground = np.array(ground)
    # Eğik (perspektif) görüntüyü düz (kuş bakışı) görüntüye çevirecek
    return cv2.getPerspectiveTransform(corners, ground.astype(np.float32))


IPM_MATRIX = build_ipm_matrix()