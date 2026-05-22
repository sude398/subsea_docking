import numpy as np

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────

#***************************************************************************************************
# Alttakiler genel kamera özellikleri, buradaki width, height, focal length, dist_coeffs, pitch_angle_rad değerleri güncellenmeli 
#***************************************************************************************************
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOCAL_LENGTH_PX = 230.0
MARKER_SIZE = 0.15

CAMERA_MATRIX = np.array([
    [FOCAL_LENGTH_PX, 0, IMAGE_WIDTH/2],
    [0, FOCAL_LENGTH_PX, IMAGE_HEIGHT/2],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.03, -0.01, 0, 0, 0], dtype=np.float32) 

# OFSET VE EŞİKLER
# Kameranın aracın merkezinden Y eksenindeki ofseti (metre).
# Örn: Kamera 25cm gerideyse 
CAMERA_OFFSET_Y = -0.25
CONFIDENCE_THRESHOLD = 20 # 4 marker'ın stabil görülme süresi (frame)

PITCH_ANGLE_RAD = np.deg2rad(90)  # Düz aşağı bakıyor
UNDERWATER_REFRACTION = 1.33

#***************************************************************************************************
#Alttakiler genel algoritmayla ve portla alakalı değerler şuanki halleri korunabilirde değişebilirde
#***************************************************************************************************
MAX_LATERAL = 0.35
MAX_YAW = 0.25
MAX_VERTICAL = 0.18

# ── FSM ve Görev Mesafeleri ──
DIST_ALIGN = 3.0       # (m) Hizalanma başlangıcı (Yaklaşma moduna geçiş sınırı)
DIST_DESCENDING = 1.00 # (m) İnişe başlama yatay hata payı (Erken İniş - Fire and Forget)
FINAL_ALIGN_DIST = 0.45# (m) Yükseklik bazlı yavaşlatma (Z ekseninde bu mesafeden sonra yavaşlar)
DIST_DOCKED = 0.15     # (m) Yanaşma tamamlandı kabul edilen dikey mesafe

PRECISION_THRESH = 0.03
HOVER_TIME = 1.2 #  1.2 saniye boyunca sabit kal
FAILSAFE_LOST = 4.0 # 4 saniye marker görünmezse: tekrar SEARCHING

SERIAL_PORT = '/dev/ttyACM0'  # Jetson üzerindeki Pixhawk portu
BAUD_RATE = 115200

# Platform ArUco IDs and diagonal pairs
PLATFORM_ARUCO_IDS = {28, 7, 19, 96}
ARUCO_DIAGONAL_PAIRS = [
    {28, 96},  # Top-Left & Bottom-Right
    {7, 19}    # Top-Right & Bottom-Left
]