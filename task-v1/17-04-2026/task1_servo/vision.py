import cv2
import cv2.aruco as aruco
import numpy as np
from builtins import range, len, int, float, abs, round, tuple # round eklendi

from config import (
    IMAGE_WIDTH, IMAGE_HEIGHT, MARKER_SIZE,
    CAMERA_MATRIX, DIST_COEFFS,
    UNDERWATER_REFRACTION
)

# ─── YENİ: GÖRSELLEŞTİRME İÇİN RENK TANIMLARI ───
COLOR_GREEN = (0, 255, 0)   # Kilitlenme/Başarı
COLOR_ORANGE = (0, 165, 255) # Tespit/Arama
COLOR_RED = (0, 0, 255)     # Hata/Uyarı
COLOR_YELLOW = (0, 255, 255) # Kamera Merkezi
COLOR_CYAN = (255, 255, 0)   # İstasyon Merkezi

class Vision:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())

        # Marker köşelerini tanımla (0,0 merkezi referans alınır)
        h = MARKER_SIZE / 2.0
        self.single_marker_obj_pts = np.array([
            [-h,  h, 0.0], [ h,  h, 0.0], [ h, -h, 0.0], [-h, -h, 0.0]
        ], dtype=np.float32)

    def compute_world(self, tvec, current_pitch_deg):
        """Kamera koordinatlarını araç/dünya koordinatlarına çevirir."""
        pitch_rad = np.deg2rad(current_pitch_deg)
        x_c, y_c, z_c = tvec[0], tvec[1], tvec[2]
        
        # Eğim ve su kırılması düzeltmesi
        x_w = x_c / 1.2  # Yatayda basit bir düzeltme çarpanı
        # z_c: kameranın derinliği, y_c: kameranın dikey ekseni
        y_w = (z_c * np.cos(pitch_rad) - y_c * np.sin(pitch_rad)) / UNDERWATER_REFRACTION
        return x_w, y_w

    def process(self, frame, current_pitch_deg):
        """ArUco tespiti, stratejik hedef belirleme ve GÖRSELLEŞTİRME yapar."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        n_detected = len(ids) if ids is not None else 0
        visible = False
        locked = False
        n_valid = 0
        target_tvec = None
        target_yaw = 0.0
        d_v = 0.0
        y_err_px = 0.0 # Dikey hata (piksel)

        # ─── 1. EKRAN MERKEZİNİ ÇİZ (SARI ARTI) ───
        cam_cx, cam_cy = int(IMAGE_WIDTH/2), int(IMAGE_HEIGHT/2)
        cv2.drawMarker(frame, (cam_cx, cam_cy), COLOR_YELLOW, cv2.MARKER_CROSS, 30, 2)

        if n_detected > 0:
            # ─── 2. TESPİT EDİLEN ARUCOLARI VE KÖŞELERİNİ ÇİZ ───
            # drawDetectedMarkers varsayılan olarak köşeleri ve ID'yi çizer.
            aruco.drawDetectedMarkers(frame, corners, ids)
            
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
                # ─── 3. STRATEJİK HEDEF MERKEZİNİ HESAPLA ───
                if n_valid == 1:
                    target_tvec = tvecs[0]
                    locked = False
                elif n_valid == 2:
                    dist_2 = np.linalg.norm(tvecs[0] - tvecs[1])
                    target_tvec = (tvecs[0] + tvecs[1]) / 2.0
                    # Diagonal mesafe kontrolü (config'den MIN_MARKER_PAIR_DIST kullanılmalı)
                    # Buraya şimdilik 0.85m sabitini koyuyorum, config'e taşınmalı.
                    if dist_2 > 0.85: 
                        locked = True
                    else:
                        locked = False
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

                # ─── 4. HESAPLANAN İSTASYON MERKEZİNİ VE VERİLERİ ÇİZ ───
                if target_tvec is not None:
                    visible = True
                    
                    # Projeksiyon: 3D hedef noktasını 2D piksel koordinatına çevir
                    # Bu sayede hayali merkezi ekranda görebiliriz.
                    # Not: tvecs[0]'ın rvec'ini kullanıyoruz, n_valid > 1 için rvec ortalaması daha iyi olabilir.
                    image_pts, _ = cv2.projectPoints(
                        target_tvec.reshape(1,3), rvecs[0], tvecs[0], 
                        CAMERA_MATRIX, DIST_COEFFS
                    )
                    
                    target_px = tuple(image_pts[0][0].astype(int))
                    
                    # İstasyon Merkezini Çiz (Turkuaz Daire)
                    cv2.circle(frame, target_px, 15, COLOR_CYAN, -1) # İçi dolu daire
                    cv2.putText(frame, "ISTASYON MERKEZI", (target_px[0]+20, target_px[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CYAN, 1)

                    # Kamera Merkezi ile İstasyon Merkezi arasındaki Çizgiyi Çiz (Turuncu)
                    cv2.line(frame, (cam_cx, cam_cy), target_px, COLOR_ORANGE, 2)
                    
                    # Dikey Hata (Servo PID'si için)
                    y_err_px = float(target_px[1] - cam_cy)

                    # Yaw hesabı
                    R, _ = cv2.Rodrigues(rvecs[0])
                    target_yaw = np.arctan2(R[1,0], R[0,0])
                    # Derinlik/Dikey mesafe (Düzeltme: abs eklendi)
                    d_v = abs(target_tvec[1]) / UNDERWATER_REFRACTION

                    # Ekrana Anlık Verileri Yaz
                    # round(d_v, 2) -> virgülden sonra 2 basamak
                    cv2.putText(frame, f"Derinlik Hata: {round(d_v, 2)}m", (10, IMAGE_HEIGHT - 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_GREEN, 2)
                    cv2.putText(frame, f"Yaw Hata: {round(np.rad2deg(target_yaw), 1)}deg", (10, IMAGE_HEIGHT - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_GREEN, 2)

        # ─── 5. KİLİTLENME DURUMUNU YAZ (SOL ÜST) ───
        lock_status = "LOCKED" if locked else "SEARCHING/ALIGNING"
        lock_color = COLOR_GREEN if locked else COLOR_ORANGE
        cv2.putText(frame, f"STATUS: {lock_status} ({n_valid} Markers)", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, lock_color, 2)

        return frame, visible, locked, n_valid, target_tvec, target_yaw, d_v, y_err_px