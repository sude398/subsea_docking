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
                    dist_3d = np.linalg.norm(tvecs[0] - tvecs[1])
                    target_tvec = (tvecs[0] + tvecs[1]) / 2.0
                    locked = dist_3d > 0.85
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

                    d_v = abs(target_tvec[1]) / UNDERWATER_REFRACTION

        return frame, visible, locked, n_valid, target_tvec, target_yaw, d_v