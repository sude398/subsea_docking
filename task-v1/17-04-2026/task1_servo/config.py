import numpy as np

# GÖRÜNTÜ & KAMERA
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
UNDERWATER_REFRACTION = 1.33
FOCAL_LENGTH_PX = 480.0 * UNDERWATER_REFRACTION 
MARKER_SIZE = 0.15
# config.py dosyasına eklenecek/güncellenecek satır:
MIN_MARKER_PAIR_DIST = 0.85  # 2 marker arası en az bu kadar mesafe olmalı (metre)

CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH_PX, 0, IMAGE_WIDTH  / 2],
    [0, FOCAL_LENGTH_PX, IMAGE_HEIGHT / 2],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.03, -0.01, 0, 0, 0], dtype=np.float32)

# HAREKET LİMİTLERİ
MAX_LATERAL  = 0.30
MAX_FORWARD  = 0.35
MAX_YAW      = 0.20

# DİNAMİK DİKEY HIZ AYARLARI
VZ_MAX = 0.25 # Uzaktayken maksimum alçalma hızı
VZ_MIN = 0.05 # Yaklaştığında minimum (hassas) iniş hızı

# DOCKING EŞİKLERİ
DIST_DESCENDING  = 0.8  
DIST_DOCKED      = 0.15 

# SERVO / PORT AYARLARI
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
CAMERA_SERVO_NUM = 9 
SERVO_PWM_MIN  = 1100  
SERVO_PWM_MAX  = 1900
SERVO_ANGLE_MAX = 75.0