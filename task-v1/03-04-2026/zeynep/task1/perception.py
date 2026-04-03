import cv2
import cv2.aruco as aruco
import numpy as np
from . import config


class MarkerDetector:
    def __init__(self):
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL),
            aruco.DetectorParameters()
        )
        # solvePnP için 3D nesne noktaları (marker düzlemde, z=0)
        h = config.MARKER_SIZE / 2
        self.obj_pts = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0]
        ], dtype=np.float32)

    def detect(self, gray):
        return self.detector.detectMarkers(gray)


class MarkerCluster:

    # Yatay mesafe (dist_h): FSM geçişleri ve lateral kontrol için
    # Dikey mesafe  (dist_v): DESCENDING aşamasında iniş kontrolü için
    # Yaw açısı     (yaw)  : Hizalama için
    # Görüntü merkezi (cx, cy): Piksel hata hesabı için döndürür.

    def __init__(self):
        self.helper = MarkerDetector()

        # Başlangıç değerleri — ilk frame'de makul bir tahmin
        self.filtered_dist_h = 3.0
        self.filtered_dist_v = 1.0
        self.filtered_yaw    = 0.0

        # EMA katsayıları: ani sıçramaları önlemek için filtreleme
        # Yüksek alpha  → yeni ölçüme daha çok güven (hızlı tepki, gürültülü)
        # Düşük alpha   → eski değere daha çok güven (yavaş tepki, düzgün)
        self.alpha_dist = 0.6
        self.alpha_yaw  = 0.5

    def compute(self, corners_raw, ids_raw):
        if ids_raw is None or len(ids_raw) == 0:
            # Marker yoksa son bilinen filtreli değerleri koru, kilit yok
            return False, 0, 0, self.filtered_dist_h, self.filtered_dist_v, self.filtered_yaw, 0

        n = len(ids_raw)
        indices_to_use = list(range(n))
        is_locked = False
        all_centers = [np.mean(c[0], axis=0) for c in corners_raw]

        # --- 1. Aşama: Kilit Mantığı ---
        if n == 1:
            is_locked = False 
        elif n == 2:
            p1, p2 = all_centers[0], all_centers[1]
            pixel_dist = np.linalg.norm(p1 - p2)
            ok, _, tvec = cv2.solvePnP(self.helper.obj_pts, corners_raw[0][0].astype(np.float32), 
                                     config.CAMERA_MATRIX, config.DIST_COEFFS)
            if ok:
                dist_m = np.linalg.norm(tvec)
                # config.FOCAL_LENGTH_PX ve 1.138 (diagonal m) kullanımı
                expected_diag_px = (1.138 * config.FOCAL_LENGTH_PX) / max(dist_m, 0.1)
                is_locked = pixel_dist > (expected_diag_px * 0.85)
        elif n == 3:
            max_d = 0
            diag_pair = (0, 1)
            for i, j in [(0, 1), (0, 2), (1, 2)]:
                d = np.linalg.norm(all_centers[i] - all_centers[j])
                if d > max_d:
                    max_d = d
                    diag_pair = (i, j)
            indices_to_use = list(diag_pair)
            is_locked = True
        elif n >= 4:
            is_locked = True

        # --- 2. Aşama: Hesaplamalar ---
        centers_to_avg, dists_h, dists_v, sin_yaws, cos_yaws = [], [], [], [], []

        for idx in indices_to_use:
            ok, rvec, tvec = cv2.solvePnP(self.helper.obj_pts, corners_raw[idx][0].astype(np.float32),
                                        config.CAMERA_MATRIX, config.DIST_COEFFS)
            if ok:
                centers_to_avg.append(all_centers[idx])
                t = tvec.flatten()
                dists_h.append(np.sqrt(t[0]**2 + t[2]**2) / config.UNDERWATER_REFRACTION)
                dists_v.append(abs(t[1]) / config.UNDERWATER_REFRACTION)
                R, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(R[1, 0], R[0, 0])
                sin_yaws.append(np.sin(yaw))
                cos_yaws.append(np.cos(yaw))

        # --- 3. Aşama: Filtreleme ---
        if dists_h:
            self.filtered_dist_h = self.alpha_dist * np.mean(dists_h) + (1 - self.alpha_dist) * self.filtered_dist_h
            self.filtered_dist_v = self.alpha_dist * np.mean(dists_v) + (1 - self.alpha_dist) * self.filtered_dist_v
        
        if sin_yaws:
            raw_yaw = np.arctan2(np.mean(sin_yaws), np.mean(cos_yaws))
            dyaw = np.arctan2(np.sin(raw_yaw - self.filtered_yaw), np.cos(raw_yaw - self.filtered_yaw))
            self.filtered_yaw += self.alpha_yaw * dyaw

        # Merkez hesabı
        if centers_to_avg:
            cx = int(np.mean([p[0] for p in centers_to_avg]))
            cy = int(np.mean([p[1] for p in centers_to_avg]))
        else:
            # PnP başarısızsa ekranın ortasını vererek sapıtmasını önle (veya son cx,cy tutulabilir)
            cx, cy = config.IMAGE_WIDTH // 2, config.IMAGE_HEIGHT // 2

        # Blind Zone Koruma: n=1 olsa bile eğer çok yakındaysak kilidi zorla aç
        if self.filtered_dist_h < config.DIST_BLIND_ZONE and n >= 1:
            is_locked = True

        return is_locked, cx, cy, self.filtered_dist_h, self.filtered_dist_v, self.filtered_yaw, n
