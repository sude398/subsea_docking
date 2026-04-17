import cv2
import cv2.aruco as aruco
import numpy as np
import logging

from config import (
    IMAGE_WIDTH, IMAGE_HEIGHT, MARKER_SIZE,
    CAMERA_MATRIX, DIST_COEFFS,
    PITCH_ANGLE_RAD, UNDERWATER_REFRACTION
)

logger = logging.getLogger(__name__)

class Vision:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())

        h = MARKER_SIZE / 2.0
        self.single_marker_obj_pts = np.array([
            [-h,  h, 0.0], [ h,  h, 0.0], [ h, -h, 0.0], [-h, -h, 0.0]
        ], dtype=np.float32)

    def _detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        n_detected = len(ids) if ids is not None else 0
        return corners, ids, n_detected

    def _estimate_poses(self, corners, n_detected):
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
        return tvecs, rvecs

    def _fuse_targets(self, tvecs):
        n_valid = len(tvecs)
        target_tvec = None
        locked = False

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

        return target_tvec, locked

    def _compute_yaw(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return yaw

    def compute_world(self, tvec):
        x = tvec[0] / 1.2
        y = (tvec[2]*np.cos(PITCH_ANGLE_RAD) - tvec[1]*np.sin(PITCH_ANGLE_RAD)) / UNDERWATER_REFRACTION
        return x, y

    def process(self, frame):
        """
        Döndürür:
            frame       : HUD çizgileri eklenmiş frame
            visible     : Hedef görülüyor mu?
            locked      : Kilitlenme sağlandı mı?
            n_valid     : Geçerli marker sayısı
            target_tvec : Hedef tvec (None olabilir)
            target_yaw  : Hedef yaw açısı
            d_v         : Dikey mesafe. (Eğer görülmüyorsa sonsuzluk/999.0 döner!)
        """
        visible = False
        locked = False
        n_valid = 0
        target_tvec = None
        target_yaw = 0.0
        
        # HATA DÜZELTMESİ: Görülememe durumunda d_v 0 olmamalı. Çok uzak kabul edilmeli.
        d_v = 999.0 

        corners, ids, n_detected = self._detect_markers(frame)

        if n_detected > 0:
            aruco.drawDetectedMarkers(frame, corners)
            tvecs, rvecs = self._estimate_poses(corners, n_detected)
            n_valid = len(tvecs)

            if n_valid > 0:
                target_tvec, locked = self._fuse_targets(tvecs)

                if target_tvec is not None:
                    visible = True
                    target_yaw = self._compute_yaw(rvecs[0])
                    d_v = abs(target_tvec[1]) / UNDERWATER_REFRACTION

        return frame, visible, locked, n_valid, target_tvec, target_yaw, d_v
