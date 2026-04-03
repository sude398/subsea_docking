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
            return False, 0, 0, self.filtered_dist_h, self.filtered_dist_v, self.filtered_yaw, 0

        centers   = [np.mean(c[0], axis=0) for c in corners_raw]
        dists_h   = []
        dists_v   = []
        sin_yaws  = []
        cos_yaws  = []

        for i, mid in enumerate(ids_raw.flatten()):
            ok, rvec, tvec = cv2.solvePnP(
                self.helper.obj_pts,
                corners_raw[i][0].astype(np.float32),
                config.CAMERA_MATRIX,
                config.DIST_COEFFS
            )

            if not ok:
                continue

            t = tvec.flatten()

            # Kamera koordinat sistemi: X sağ, Y aşağı, Z ileri
            # Yatay mesafe: XZ düzleminde (derinlik + sağ-sol)
            # Dikey mesafe: Y ekseni (kameradan marker'a yukarı/aşağı fark)
            raw_dist_h = np.sqrt(t[0]**2 + t[2]**2) / config.UNDERWATER_REFRACTION
            raw_dist_v = abs(t[1])                   / config.UNDERWATER_REFRACTION

            dists_h.append(raw_dist_h)
            dists_v.append(raw_dist_v)

            # Yaw: marker'ın Z ekseni etrafındaki dönüşü
            R, _ = cv2.Rodrigues(rvec)
            yaw  = np.arctan2(R[1, 0], R[0, 0])
            sin_yaws.append(np.sin(yaw))
            cos_yaws.append(np.cos(yaw))

        # ── DIST EMA FİLTRESİ ──────────────────────────────────────────
        if dists_h:
            self.filtered_dist_h = (
                self.alpha_dist * np.mean(dists_h) +
                (1 - self.alpha_dist) * self.filtered_dist_h
            )
            self.filtered_dist_v = (
                self.alpha_dist * np.mean(dists_v) +
                (1 - self.alpha_dist) * self.filtered_dist_v
            )

        # ── YAW CIRCULAR EMA FİLTRESİ ──────────────────────────────────
        # Düz ortalama yerine circular average kullan:
        # 350° ve 10°'nin ortalaması 180° değil, 0° olmalı.
        if sin_yaws:
            raw_yaw = np.arctan2(np.mean(sin_yaws), np.mean(cos_yaws))
            dyaw = np.arctan2(
                np.sin(raw_yaw - self.filtered_yaw),
                np.cos(raw_yaw - self.filtered_yaw)
            )
            self.filtered_yaw += self.alpha_yaw * dyaw

        # ── PLATFORM MERKEZİ (piksel) ───────────────────────────────────
        cx = int(np.mean([p[0] for p in centers]))
        cy = int(np.mean([p[1] for p in centers]))

        # ── LOCK KRİTERİ ───────────────────────────────────────────────
        # Uzakta 2+ marker şart (güvenilir pose).
        # Yakında (blind zone) 1 marker yeterli — kamera açısından dolayı
        # bazı marker'lar frame dışına çıkar.
        n = len(ids_raw)
        if self.filtered_dist_h > config.DIST_BLIND_ZONE:
            is_locked = n >= 2
        else:
            is_locked = n >= 1

        return is_locked, cx, cy, self.filtered_dist_h, self.filtered_dist_v, self.filtered_yaw, n
