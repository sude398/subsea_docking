import cv2
import cv2.aruco as aruco
import numpy as np
from . import config


class MarkerDetector:
    def __init__(self):
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters()
        )
        # homografi matrisi, ipm için gerekli
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
    def __init__(self):
        self.helper = MarkerDetector()

        self.filtered_dist = 3.0
        self.filtered_yaw  = 0.0
        # ani değişimleri önlemek için yeni verinin %60'ını alarak eski değerle birleştiriyoruz (EMA)
        self.alpha_dist = 0.6
        self.alpha_yaw  = 0.5

    def compute(self, corners, ids, corners_raw, prev_dist):

        if ids is None or len(ids) == 0:
            return False, 0, 0, self.filtered_dist, self.filtered_yaw, 0

        raw_dict = {}
        if corners_raw is not None:
            for i, rid in enumerate(ids.flatten()):
                raw_dict[rid] = corners_raw[i]

        centers = [np.mean(c[0], axis=0) for c in corners]
        # her markerın sin ve cos açıları tutulur
        dists = []
        sin_yaws = []
        cos_yaws = []

        for i, mid in enumerate(ids.flatten()):

            if mid not in raw_dict:
                continue

            #mesafe çizimi
            # rvec ve tvec, marker'ın kamera koordinatlarında pozisyonunu verir (dik ve eğik mesafe)
            ok, rvec, tvec = cv2.solvePnP(
                self.helper.obj_pts,
                raw_dict[mid][0].astype(np.float32),
                config.CAMERA_MATRIX,
                config.DIST_COEFFS
            )

            if ok:
                dist = np.linalg.norm(tvec) / config.UNDERWATER_REFRACTION
                dists.append(dist)

                R, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(R[1, 0], R[0, 0])

                sin_yaws.append(np.sin(yaw))
                cos_yaws.append(np.cos(yaw))


        # DIST FILTER
        if dists:
            raw_dist = np.mean(dists)
            self.filtered_dist = (
                self.alpha_dist * raw_dist +
                (1 - self.alpha_dist) * self.filtered_dist
            )

        # YAW FILTER (EMA (Exponential Moving Average) + circular)
        if sin_yaws and len(sin_yaws) > 0:
            raw_yaw = np.arctan2(np.mean(sin_yaws), np.mean(cos_yaws))

            # EMA (açı farkını normalize ederek)
            # mevcut filtrelenmiş açı ile yeni ölçülen açı arasındaki en kısa farkı hesaplar (fark 2 derece ise araç 358 derece dönmez)
            dyaw = np.arctan2(
                np.sin(raw_yaw - self.filtered_yaw),
                np.cos(raw_yaw - self.filtered_yaw)
            )

            self.filtered_yaw += self.alpha_yaw * dyaw


        # Platform merkezi
        mx = int(np.mean([p[0] for p in centers]))
        my = int(np.mean([p[1] for p in centers]))

        # LOCK CRITERIA
        is_locked = len(ids) >= 2

        return is_locked, mx, my, self.filtered_dist, self.filtered_yaw, len(ids)