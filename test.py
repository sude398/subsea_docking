import time
import sys
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np

# ── DURUM MAKİNESİ ────────────────────────────────────────────────────────────
class ROVState(Enum):
    SEARCHING   = auto() 
    APPROACHING = auto() 
    PRECISION   = auto() 
    DOCKED      = auto() 

# ── MATEMATİK VE FİLTRELER ───────────────────────────────────────────────────
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_val = None

    def update(self, new_val):
        if self.last_val is None:
            self.last_val = new_val
            return new_val
        filtered = self.alpha * new_val + (1.0 - self.alpha) * self.last_val
        self.last_val = filtered
        return filtered

class PID:
    def __init__(self, kp, ki, kd, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit  
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def compute(self, current_val, target_val):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0: dt = 0.033

        error = target_val - current_val
        p_term = self.kp * error
        self.integral = np.clip(self.integral + error * dt, -self.limit, self.limit)
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.last_error) / dt
        
        output = p_term + i_term + d_term
        self.last_error = error
        self.last_time = now
        
        return np.clip(output, -self.limit, self.limit)

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

# ── ANA ALGORİTMA SINIFI ─────────────────────────────────────────────────────
class AutonomousDockingSystem:
    def __init__(self, source=0):
        self.cap = cv2.VideoCapture(source)
        
        self.WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if self.WIDTH == 0 or self.HEIGHT == 0:
            self.WIDTH, self.HEIGHT = 1920, 1080 

        # 2. TAC 2026 Platform 3D Koordinatları
        X_REF = (800 / 2 - 35 - 75) / 1000.0
        Y_REF = (1200 / 2 - 35 - 75) / 1000.0
        self.target_3d_points = {
            28: (-X_REF, -Y_REF, 0.0), 7:  ( X_REF, -Y_REF, 0.0),
            19: (-X_REF,  Y_REF, 0.0), 96: ( X_REF,  Y_REF, 0.0)
        }
        self.target_ids = list(self.target_3d_points.keys())

        # YENİ: Etiketlerin fiziksel boyutu (15 cm varsayıldı, gerekiyorsa değiştirin)
        self.MARKER_SIZE = 0.15 
        self.half_size = self.MARKER_SIZE / 2.0

        # 3. Kamera Matrisi
        self.camera_matrix = np.array([
            [900.0, 0.0, self.WIDTH/2], 
            [0.0, 900.0, self.HEIGHT/2], 
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([[0.1, -0.2, 0.0, 0.0, 0.0]], dtype=np.float32)

        # 4. ArUco Kurulumu
        if hasattr(aruco, 'ArucoDetector'):
            self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL))
            self._detect = lambda gray: self.detector.detectMarkers(gray)
        else:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_params = aruco.DetectorParameters_create()
            self._detect = lambda gray: aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # 5. PID Kontrolcüler
        self.pid_surge = PID(kp=0.8, ki=0.01, kd=0.1, limit=1.0)
        self.pid_sway  = PID(kp=1.0, ki=0.01, kd=0.2, limit=1.0)
        self.pid_heave = PID(kp=1.2, ki=0.02, kd=0.2, limit=1.0)
        self.pid_yaw   = PID(kp=0.5, ki=0.0,  kd=0.05, limit=1.0)

        self.lpf_x = LowPassFilter(0.7)
        self.lpf_y = LowPassFilter(0.7)
        self.lpf_z = LowPassFilter(0.8)
        self.lpf_yaw = LowPassFilter(0.85)

        self.state = ROVState.SEARCHING
        self.target_stop_distance = 0.45
        self.locked_time = None
        self.hold_duration = 5.0
        
        self.hud_data = {'dist': 0.0, 'cx': self.WIDTH//2, 'cy': self.HEIGHT//2, 'tags': 0}

    def reset_pids(self):
        self.pid_surge.reset()
        self.pid_sway.reset()
        self.pid_heave.reset()
        self.pid_yaw.reset()

    def process_frame(self, frame):
        # EĞER KAMERA TERS İSE:
        # frame = cv2.rotate(frame, cv2.ROTATE_180)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        
        img_pts, obj_pts = [], []
        surge = sway = heave = yaw = 0.0
        num_detected = 0

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                if m_id in self.target_ids:
                    num_detected += 1
                    cx_3d, cy_3d, _ = self.target_3d_points[m_id]
                    
                    # KAYMAYI ÇÖZEN KISIM: 
                    # Görünen her etiketin 4 köşesini platform merkezine (0,0,0) göre uzayda tanımlıyoruz.
                    marker_3d_corners = [
                        [cx_3d - self.half_size, cy_3d - self.half_size, 0.0], # Top-Left
                        [cx_3d + self.half_size, cy_3d - self.half_size, 0.0], # Top-Right
                        [cx_3d + self.half_size, cy_3d + self.half_size, 0.0], # Bottom-Right
                        [cx_3d - self.half_size, cy_3d + self.half_size, 0.0]  # Bottom-Left
                    ]
                    obj_pts.extend(marker_3d_corners)
                    img_pts.extend(corners[i][0].tolist())

        self.hud_data['tags'] = num_detected

        if self.state == ROVState.DOCKED:
            surge = sway = heave = yaw = 0.0
            
        elif num_detected == 0:
            self.state = ROVState.SEARCHING
            self.reset_pids()
            self.locked_time = None
            surge = 0.2  
            
        elif num_detected >= 1:
            # Sadece 1 etiket (4 köşe) bile görsek solvePnP çalışır ve gerçek platform merkezini bulur!
            success, rvec, tvec = cv2.solvePnP(
                np.array(obj_pts, dtype=np.float32), 
                np.array(img_pts, dtype=np.float32), 
                self.camera_matrix, self.dist_coeffs, 
                flags=cv2.SOLVEPNP_SQPNP
            )
            
            if success and tvec[2][0] > 0:
                # 3D Orijini (0,0,0) yani platform merkezini ekrana 2D olarak geri yansıt (ProjectPoints)
                center_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, self.camera_matrix, self.dist_coeffs)
                self.hud_data['cx'] = int(center_2d[0][0][0])
                self.hud_data['cy'] = int(center_2d[0][0][1])

                raw_x, raw_y, raw_z = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                rmat, _ = cv2.Rodrigues(rvec)
                raw_yaw = float(cv2.RQDecomp3x3(rmat)[0][2])
                
                f_x = self.lpf_x.update(raw_x)
                f_y = self.lpf_y.update(raw_y)
                f_z = self.lpf_z.update(raw_z)
                f_yaw = self.lpf_yaw.update(raw_yaw)
                
                self.hud_data['dist'] = f_z
                
                if num_detected >= 3:
                    self.state = ROVState.PRECISION
                    surge = -self.pid_surge.compute(f_z, self.target_stop_distance)
                    sway  = -self.pid_sway.compute(f_x, 0.0)
                    heave = self.pid_heave.compute(f_y, 0.0)
                    yaw   = -self.pid_yaw.compute(f_yaw, 0.0)

                    if abs(f_x) < 0.05 and abs(f_y) < 0.05 and abs(f_yaw) < 5.0 and f_z < (self.target_stop_distance + 0.1):
                        if self.locked_time is None: self.locked_time = time.time()
                        if (time.time() - self.locked_time) >= self.hold_duration:
                            self.state = ROVState.DOCKED
                    else:
                        self.locked_time = None
                else:
                    self.state = ROVState.APPROACHING
                    self.locked_time = None
                    # Yansıtılan gerçek merkeze göre 2D yaklaş
                    cx, cy = self.hud_data['cx'], self.hud_data['cy']
                    sway  = ((cx - self.WIDTH/2) / (self.WIDTH/2)) * 0.4
                    heave = -((cy - self.HEIGHT/2) / (self.HEIGHT/2)) * 0.3
                    yaw   = -((cx - self.WIDTH/2) / (self.WIDTH/2)) * 0.3
                    surge = 0.3 

        self.draw_hud(frame, surge, sway, heave, yaw)
        output_vector = {"surge": surge, "sway": sway, "heave": heave, "yaw": yaw}
        return frame, output_vector

    def draw_hud(self, frame, surge, sway, heave, yaw):
        h, w = frame.shape[:2]
        color = (0, 255, 0) if self.state in [ROVState.PRECISION, ROVState.DOCKED] else (0, 165, 255) if self.state == ROVState.APPROACHING else (0, 0, 255)

        if self.state != ROVState.SEARCHING:
            cx, cy = self.hud_data['cx'], self.hud_data['cy']
            cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 30, 2)
            cv2.line(frame, (w//2, h//2), (cx, cy), (200, 200, 0), 1)

        cv2.circle(frame, (w//2, h//2), 15, (255, 255, 0), 1)

        cv2.putText(frame, f"STATE: {self.state.name}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 4)
        cv2.putText(frame, f"TAGS : {self.hud_data['tags']}", (40, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(frame, f"DIST : {self.hud_data['dist']:.2f}m", (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

        panel_y1 = h - 180
        cv2.rectangle(frame, (20, panel_y1), (800, h - 20), (0, 0, 0), -1)
        
        hud_line1 = f"SURGE: {surge:+.2f} | SWAY : {sway:+.2f}"
        hud_line2 = f"HEAVE: {heave:+.2f} | YAW  : {yaw:+.2f}"
        cv2.putText(frame, hud_line1, (40, panel_y1 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
        cv2.putText(frame, hud_line2, (40, panel_y1 + 130), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)

    def run(self):
        print("▶ Otonom Sistem Başlatıldı. Çıkmak için 'q' tuşuna basın.")
        while True:
            ret, frame = self.cap.read()
            if not ret: break

            processed_frame, outputs = self.process_frame(frame)
            
            h, w = processed_frame.shape[:2]
            scale_percent = 70  
            new_width = int(w * scale_percent / 100)
            new_height = int(h * scale_percent / 100)
            display = cv2.resize(processed_frame, (new_width, new_height))
            
            cv2.imshow("TAC 2026 - Generic Docking", display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    try: source = int(source)
    except ValueError: pass
    
    system = AutonomousDockingSystem(source)
    system.run()