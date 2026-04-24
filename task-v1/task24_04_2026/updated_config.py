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
# Kamera aracın en önündeyse ve araç merkezi 25cm gerideyse: -0.35
CAMERA_OFFSET_Y = -0.35 
CONFIDENCE_THRESHOLD = 20 # 4 marker'ın stabil görülme süresi (frame)

PITCH_ANGLE_RAD = np.deg2rad(45) 
UNDERWATER_REFRACTION = 1.33

#***************************************************************************************************
#Alttakiler genel algoritmayla ve portla alakalı değerler şuanki halleri korunabilirde değişebilirde
#***************************************************************************************************
MAX_LATERAL = 0.35
MAX_YAW = 0.25
MAX_VERTICAL = 0.18

DIST_ALIGN = 3.0 # 3 metreye gelince: SEARCHING → ALIGNING
DIST_DESCENDING = 0.55 # 0.55 metreye merkezi geçince ALIGNING → DESCENDING
FINAL_ALIGN_DIST = 0.45 # 0.45 metreye gelince: DESCENDING → FINAL_ALIGNING
DIST_DOCKED = 0.15 # 0.15 metreye gelince: FINAL_ALIGNING → DOCKED

PRECISION_THRESH = 0.03
HOVER_TIME = 1.2 #  1.2 saniye boyunca sabit kal
FAILSAFE_LOST = 4.0 # 4 saniye marker görünmezse: tekrar SEARCHING

# Yakın mesafede kamera FOV'undan çıkan marker kaybı beklenen bir durumdur; bu eşiklerle kör iniş devreye girer
BLIND_DESCENT_DIST = 1.5      # Bu yatay mesafenin altında FOV kaybı beklenir (m)
BLIND_DESCENT_TIMEOUT = 8.0   # Son bilinen poz ile devam edilecek maksimum süre (s)

SERIAL_PORT = '/dev/ttyACM0'  # Jetson üzerindeki Pixhawk portu
BAUD_RATE = 115200