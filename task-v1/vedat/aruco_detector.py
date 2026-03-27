import cv2
import numpy as np
import time

class PID:
    def __init__(self, kp, ki, kd, min_val, max_val, name="PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_val = min_val
        self.max_val = max_val
        self.name = name

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def compute(self, current_val, target_val):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0: dt = 0.033  # fallback

        error = target_val - current_val
        
        # Proportional
        p_term = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-200, min(200, self.integral)) # windup protection
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.last_error) / dt
        
        output = p_term + i_term + d_term
        
        self.last_error = error
        self.last_time = now
        
        return max(self.min_val, min(self.max_val, output))

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha  # 0.0 to 1.0 (higher = more weight to new data)
        self.last_val = None

    def filter(self, new_val):
        if self.last_val is None:
            self.last_val = new_val
            return new_val
        
        filtered = self.alpha * new_val + (1.0 - self.alpha) * self.last_val
        self.last_val = filtered
        return filtered

class ArUcoDetector:
    def __init__(self):
        # TAC 2026 Platform Dimensions
        # These are used for solvePnP to estimate distance and pose
        X_REF = (800 / 2 - 35 - 75) / 1000.0
        Y_REF = (1200 / 2 - 35 - 75) / 1000.0
        self.target_3d_points = {
            28: (-X_REF, -Y_REF, 0.0), 7:  ( X_REF, -Y_REF, 0.0),
            19: (-X_REF,  Y_REF, 0.0), 96: ( X_REF,  Y_REF, 0.0)
        }
        self.target_ids = list(self.target_3d_points.keys())

        # Camera Matrix (1920x1080)
        self.camera_matrix = np.array([
            [1536.0, 0.0, 960.0], 
            [0.0, 1536.0, 540.0], 
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([[0.1, -0.2, 0.0, 0.0, 0.0]], dtype=np.float32)

        # ArUco Configuration
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # PID Controllers
        # Surge (Move towards target), Sway (Left/Right centering), Heave (Up/Down centering), Yaw (Orientation)
        # YENİ (Yumuşak ve Sakin) DEĞERLER
        self.pid_surge = PID(kp=80.0,  ki=1.0, kd=15.0, min_val=-300, max_val=300, name="Surge")
        self.pid_sway  = PID(kp=100.0, ki=1.0, kd=20.0, min_val=-300, max_val=300, name="Sway")
        self.pid_heave = PID(kp=120.0, ki=2.0, kd=20.0, min_val=-300, max_val=300, name="Heave")
        self.pid_yaw   = PID(kp=25.0,  ki=0.0, kd=1.0,  min_val=-200, max_val=200, name="Yaw")

        # Low Pass Filters for Pose Smoothing
        self.lpf_x = LowPassFilter(0.7)
        self.lpf_y = LowPassFilter(0.7)
        self.lpf_z = LowPassFilter(0.8)
        self.lpf_yaw = LowPassFilter(0.85)

        self.target_stop_distance = 0.45

    def detect_markers(self, frame):
        """Detect ArUco markers and return detected points for solvePnP"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        detected_info = {}
        img_pts, obj_pts = [], []

        if ids is not None:
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                if m_id in self.target_ids:
                    cx, cy = np.mean(corners[i][0][:, 0]), np.mean(corners[i][0][:, 1])
                    detected_info[m_id] = (cx, cy)
                    img_pts.append([cx, cy])
                    obj_pts.append(self.target_3d_points[m_id])
        
        return detected_info, img_pts, obj_pts, corners

    def calculate_pnp_pose(self, img_pts, obj_pts):
        """Calculate 6-DOF pose using solvePnP"""
        if len(img_pts) < 3:
            return None
        
        success, rvec, tvec = cv2.solvePnP(
            np.array(obj_pts, dtype=np.float32), 
            np.array(img_pts, dtype=np.float32), 
            self.camera_matrix, self.dist_coeffs, 
            flags=cv2.SOLVEPNP_SQPNP
        )
        
        if success:
            raw_x, raw_y, raw_z = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
            rmat, _ = cv2.Rodrigues(rvec)
            raw_yaw = float(cv2.RQDecomp3x3(rmat)[0][2])
            
            f_x = self.lpf_x.filter(raw_x)
            f_y = self.lpf_y.filter(raw_y)
            f_z = self.lpf_z.filter(raw_z)
            f_yaw = self.lpf_yaw.filter(raw_yaw)
            
            return f_x, f_y, f_z, f_yaw
        return None

    def get_centroid_centering(self, detected_info):
        """Simple centroid-based centering for APPROACHING mode"""
        if not detected_info:
            return None
        
        # Diagonal logic as seen in original script
        if 28 in detected_info and 96 in detected_info:
            mx = (detected_info[28][0] + detected_info[96][0]) / 2
            my = (detected_info[28][1] + detected_info[96][1]) / 2
        elif 7 in detected_info and 19 in detected_info:
            mx = (detected_info[7][0] + detected_info[19][0]) / 2
            my = (detected_info[7][1] + detected_info[19][1]) / 2
        else:
            mx = np.mean([p[0] for p in detected_info.values()])
            my = np.mean([p[1] for p in detected_info.values()])
            
        return mx, my

    def compute_pid_outputs(self, f_x, f_y, f_z, f_yaw):
        """Compute PID corrected outputs based on filtered pose"""
        surge = int(self.pid_surge.compute(f_z, self.target_stop_distance))
        sway  = int(self.pid_sway.compute(f_x, 0.0))
        
        h_offset = self.pid_heave.compute(f_y, 0.0)
        heave = int(h_offset) # We'll handle the base value (e.g. 1500) in the node
        
        yaw = int(self.pid_yaw.compute(f_yaw, 0.0))
        
        return surge, sway, heave, yaw

    def reset_pids(self):
        """Reset all PID controllers to prevent integral windup"""
        self.pid_surge.reset()
        self.pid_sway.reset()
        self.pid_heave.reset()
        self.pid_yaw.reset()
