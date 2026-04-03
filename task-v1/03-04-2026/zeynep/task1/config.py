import numpy as np
import cv2

# ─────────────────────────────────────────────
# GÖRÜNTÜ & KAMERA
# ─────────────────────────────────────────────
IMAGE_WIDTH     = 1920
IMAGE_HEIGHT    = 1080
FOCAL_LENGTH_PX = 1080.0
MARKER_SIZE     = 0.15      # marker kenar uzunluğu (m)

CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH_PX, 0, IMAGE_WIDTH / 2],
    [0, FOCAL_LENGTH_PX, IMAGE_HEIGHT / 2],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.03, -0.01, 0, 0, 0], dtype=np.float32)

# Kamera ileriye bakıyor, 30° aşağı eğik
CAMERA_PITCH_DEG = -30.0

# Kamera, araç gövde merkezinin 50cm önünde (ön kenarda)
# Kamera merkezi platforma hizalandığında araç 50cm geride duruyor.
CAMERA_FORWARD_OFFSET_M = 0.50

# Suyun altında ışık kırılması
UNDERWATER_REFRACTION = 1.33

# HIZ LİMİTLERİ
MAX_LATERAL  = 0.35     # m/s yatay (ileri/geri, sağ/sol)
MAX_YAW      = 0.25     # rad/s dönüş
MAX_VERTICAL = 0.18     # m/s dikey iniş

# KONTROL PARAMETRELERİ
# Hedef bu kadar piksel içindeyse hata sıfır say
DEADZONE_PX = 20

# Thruster bu değerin altında güç alırsa komut gönderme
THRUSTER_DEADBAND = 0.10 # %10 demek

# ─────────────────────────────────────────────
# FSM MESAFELERİ  (yatay mesafe, metre)
# ─────────────────────────────────────────────
DIST_ALIGN      = 3.0   # bu mesafede yaw hizalamaya başla
DIST_DESCENDING = 0.6   # bu mesafede dikey inişe geç
DIST_BLIND_ZONE = 0.40  # bu mesafeden yakında kamera platformu göremeyebilir,
                         # lock kaybı normaldir — FSM bunu tolere eder
DIST_DOCKED     = 0.15  # bu mesafede "kondu" kabul et ve dur

# ─────────────────────────────────────────────
# FAILSAFE
# ─────────────────────────────────────────────
FAILSAFE_LOST = 5.0     # 5 sn hedef görülmezse FAILSAFE

# ─────────────────────────────────────────────
# MAVLink
# ─────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
