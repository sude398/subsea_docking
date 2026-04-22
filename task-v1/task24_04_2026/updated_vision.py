from builtins import print

import cv2
import cv2.aruco as aruco
import numpy as np

from config import (
    IMAGE_WIDTH, IMAGE_HEIGHT, MARKER_SIZE,
    CAMERA_MATRIX, DIST_COEFFS,
    PITCH_ANGLE_RAD, UNDERWATER_REFRACTION
)


# ─────────────────────────────────────────────
# VISION
# ─────────────────────────────────────────────
class Vision:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())

        h = MARKER_SIZE / 2.0
        self.single_marker_obj_pts = np.array([
            [-h,  h, 0.0], [ h,  h, 0.0], [ h, -h, 0.0], [-h, -h, 0.0]
        ], dtype=np.float32)

    def compute_world(self, tvec):
        x = tvec[0] / 1.2
        y = (tvec[2]*np.cos(PITCH_ANGLE_RAD) - tvec[1]*np.sin(PITCH_ANGLE_RAD)) / UNDERWATER_REFRACTION
        return x, y

    def process(self, frame):
        """
        Verilen frame üzerinde ArUco tespiti ve pose hesaplama yapar.

        Döndürür:
            frame       : HUD çizgileri eklenmiş frame
            visible     : En az 1 geçerli marker tespit edildi mi
            locked      : Yeterli marker sayısına göre kilitlenme durumu
            n_valid     : Geçerli marker sayısı
            target_tvec : Hedef translation vektörü (None olabilir)
            target_yaw  : Hedef yaw açısı (radyan)
            d_h         : Yatay mesafe (metre)
            d_v         : Dikey mesafe (metre)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        n_detected = len(ids) if ids is not None else 0
        visible = False
        locked = False
        n_valid = 0
        target_tvec = None
        target_yaw = 0.0
        d_h, d_v = 0.0, 0.0

        if n_detected > 0:
            aruco.drawDetectedMarkers(frame, corners)
            tvecs, rvecs = [], []

            for i in range(n_detected):
                ok, r, t = cv2.solvePnP(
                    self.single_marker_obj_pts,
                    corners[i][0].astype(np.float32),
                    CAMERA_MATRIX, DIST_COEFFS
                )
                if ok:
                    tvecs.append(t.flatten())
                    rvecs.append(r)

            n_valid = len(tvecs)

            if n_valid > 0:
                if n_valid == 1:
                    target_tvec = tvecs[0]
                    locked = False
                
                elif n_valid == 2:
                    # İki marker arasındaki 3 boyutlu fark vektörü
                    diff = tvecs[0] - tvecs[1]
                    dist_3d = np.linalg.norm(diff)
                    
                    # Eksensel farklar (Mutlak değer)
                    dx = abs(diff[0])
                    dy = abs(diff[1])
                    
                    # Hedef: İki marker'ın tam orta noktası
                    target_tvec = (tvecs[0] + tvecs[1]) / 2.0
                    
                    # DİYAGONAL KONTROLÜ:
                    # Hem X hem de Y ekseninde belirgin bir fark olmalı (Örn: marker boyutunun yarısı kadar)
                    is_diagonal = dx > (MARKER_SIZE * 0.5) and dy > (MARKER_SIZE * 0.5)
                    
                    if is_diagonal and dist_3d > 0.85:
                        locked = True  # Diyagonal ve mesafe uygunsa kilitlen
                        print(">>> 2 Marker: Diyagonal Kilit Sağlandı")
                    else:
                        locked = False # Diyagonal değilse kilitlenme, 3. marker aramaya devam et (Approaching'e geçmez)
                        print(">>> 2 Marker: Diyagonal değil, 3. marker bekleniyor...")
                
                elif n_valid == 3:
                    max_d = 0
                    pair = (0, 1)
                    for i in range(3):
                        for j in range(i+1, 3):
                            d = np.linalg.norm(tvecs[i] - tvecs[j])
                            if d > max_d:
                                max_d = d
                                pair = (i, j)
                    target_tvec = (tvecs[pair[0]] + tvecs[pair[1]]) / 2.0
                    locked = True
                
                elif n_valid >= 4:
                    target_tvec = np.mean(tvecs[:4], axis=0)
                    locked = True

                if target_tvec is not None:
                    visible = True

                    R, _ = cv2.Rodrigues(rvecs[0])
                    target_yaw = np.arctan2(R[1,0], R[0,0])

                    d_v = abs(target_tvec[2]*np.cos(PITCH_ANGLE_RAD) - target_tvec[1]*np.sin(PITCH_ANGLE_RAD)) / UNDERWATER_REFRACTION

                    # 3D merkez noktasını 2D ekrana iz düşür (Project 3D to 2D)
                    # target_tvec zaten kamera koordinatlarında olduğu için rvec ve tvec sıfır verilir
                    image_pts, _ = cv2.projectPoints(
                        np.array([target_tvec], dtype=np.float32), 
                        np.zeros((3,1)), np.zeros((3,1)), 
                        CAMERA_MATRIX, DIST_COEFFS
                    )
                    center_px = tuple(image_pts[0][0].astype(int))
                    
                    # Ekrana bir hedef dairesi ve artı işareti çiz
                    cv2.circle(frame, center_px, 12, (0, 0, 255), 2) # Kırmızı daire
                    cv2.line(frame, (center_px[0]-15, center_px[1]), (center_px[0]+15, center_px[1]), (0, 0, 255), 2)
                    cv2.line(frame, (center_px[0], center_px[1]-15), (center_px[0], center_px[1]+15), (0, 0, 255), 2)
                    
                    # Merkezin yanına anlık kamera koordinatlarını yaz
                    cv2.putText(frame, f"CAM_XYZ: {target_tvec[0]:.2f}, {target_tvec[1]:.2f}, {target_tvec[2]:.2f}", 
                                (center_px[0] + 15, center_px[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        return frame, visible, locked, n_valid, target_tvec, target_yaw, d_v