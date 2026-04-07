import os
import time
import threading
from enum import Enum, auto
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Point
from pymavlink import mavutil

os.environ['MAVLINK20'] = '1'

# ── PARAMETRELER ──────────────────────────────────────────────────────────────
IMAGE_WIDTH      = 1920
IMAGE_HEIGHT     = 1080
FOCAL_LENGTH_PX  = 1080.0
CAMERA_MATRIX    = np.array([[FOCAL_LENGTH_PX, 0, 960], [0, FOCAL_LENGTH_PX, 540], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS      = np.array([-0.03, -0.01, 0, 0], dtype=np.float32)

MARKER_SIZE      = 0.15
half_size        = MARKER_SIZE / 2.0

# Platform Etiket Koordinatları (0,0,0 Merkezine Göre)
X_REF = (800 / 2 - 35 - 75) / 1000.0
Y_REF = (1200 / 2 - 35 - 75) / 1000.0
TARGET_3D_POINTS = {
    28: (-X_REF, -Y_REF, 0.0), 7:  ( X_REF, -Y_REF, 0.0),
    19: (-X_REF,  Y_REF, 0.0), 96: ( X_REF,  Y_REF, 0.0)
}
TARGET_IDS = list(TARGET_3D_POINTS.keys())

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200

# ── DURUM MAKİNESİ ────────────────────────────────────────────────────────────
class ROVState(Enum):
    CONNECTING = auto()
    ARMING     = auto()
    SEARCHING  = auto()
    ALIGNING   = auto()
    DOCKING    = auto()
    TOUCHDOWN  = auto()

# ── FİLTRE VE PID ─────────────────────────────────────────────────────────────
class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
    def update(self, value):
        self.data.append(value)
        return sum(self.data) / len(self.data)
    def get(self):
        if not self.data: return 0.0
        return sum(self.data) / len(self.data)
    def reset(self):
        self.data.clear()

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral, self.last_error = 0.0, 0.0
        self.last_time = time.time()
    def update(self, error):
        dt = time.time() - self.last_time
        if dt <= 0: dt = 0.033
        self.integral = np.clip(self.integral + error * dt, -self.limit, self.limit)
        derivative = (error - self.last_error) / dt
        self.last_error, self.last_time = error, time.time()
        return np.clip(
            (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative),
            -self.limit, self.limit
        )

# ── ANA DÜĞÜM ─────────────────────────────────────────────────────────────────
class DownwardDockingNode(Node):
    def __init__(self):
        super().__init__('downward_docking_node')

        self.declare_parameter('video_source',    0)
        self.declare_parameter('kp',              0.0012)
        self.declare_parameter('ki',              0.0001)
        self.declare_parameter('kd',              0.0004)
        self.declare_parameter('dist_touchdown',  0.45)
        self.declare_parameter('deadzone_px',     15)
        self.declare_parameter('vz_search',       0.08)
        self.declare_parameter('vz_dock_far',    -0.12)
        self.declare_parameter('vz_dock_near',   -0.05)
        self.declare_parameter('lost_timeout',    1.5)
        
        # YENİ: Burnunu aşağı eğme parametresi (Pitch)
        # 0.0 = Düz, Negatif değerler (-) burnunu aşağı eğer, Pozitifler yukarı kaldırır.
        self.declare_parameter('pitch_tilt',     -0.35) 

        self.video_source = self.get_parameter('video_source').value

        # ── Kamera ──
        self.cap = cv2.VideoCapture(self.video_source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,   IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  IMAGE_HEIGHT)

        # ── ROS ──
        self.pub_processed_img = self.create_publisher(Image, '/aruco/image_processed', qos_profile_sensor_data)
        self.pub_target_center = self.create_publisher(Point, '/aruco/target_center', 10)
        self.bridge   = CvBridge()
        
        if hasattr(aruco, 'ArucoDetector'):
            self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL))
            self._detect = lambda gray: self.detector.detectMarkers(gray)
        else:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_params = aruco.DetectorParameters_create()
            self._detect = lambda gray: aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # ── Durum ──
        self.locked           = False
        self.last_locked_time = time.time()
        self.current_dist     = 2.0
        self.last_cx          = IMAGE_WIDTH  // 2
        self.last_cy          = IMAGE_HEIGHT // 2
        self.state            = ROVState.CONNECTING
        self.armed            = False
        self.num_detected     = 0

        # ── Filtreler ──
        self.filter_cx   = MovingAverageFilter(window_size=5)
        self.filter_cy   = MovingAverageFilter(window_size=5)
        self.filter_dist = MovingAverageFilter(window_size=5)

        # ── PID ──
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid_surge = PID(kp, ki, kd, 0.5)
        self.pid_sway  = PID(kp, ki, kd, 0.5)

        # ── MAVLink ──
        self.master = None
        self._connect_pixhawk()

        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb_thread.start()

        self.create_timer(0.033, self.main_loop)
        self.get_logger().info(f"⚓ Downward Node: Kamera {self.video_source} üzerinden başlatıldı. (Pitch Kontrollü)")

    # ── YARDIMCI METODLAR ─────────────────────────────────────────────────────

    def _update_parameters(self):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        for pid in (self.pid_surge, self.pid_sway):
            pid.kp, pid.ki, pid.kd = kp, ki, kd

    def _connect_pixhawk(self):
        try:
            self.master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=255)
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info("✅ Pixhawk Bağlantısı Kuruldu!")
            self._set_param("ARMING_CHECK", 0)
            self._set_param("FS_GCS_EN",    0)
            self._set_mode("STABILIZE") # ArduSub'da Pitch açısı korumak için STABILIZE gerekir
        except Exception as e:
            self.get_logger().error(f"❌ Pixhawk Hatası: {e}")

    def _set_param(self, name, value):
        if self.master:
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                name.encode('utf-8'), float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

    def _set_mode(self, mode):
        if self.master:
            try:
                mode_id = self.master.mode_mapping()[mode]
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
            except: pass

    def _heartbeat_loop(self):
        while rclpy.ok():
            if self.master:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            time.sleep(1.0)

    # YENİ: 6 Eksenli RC Override Kontrolü
    def send_rc_override(self, vx, vy, vz, vyaw, vpitch=0.0):
        if self.master:
            # Gelen -1.0 ile 1.0 arasındaki değerleri 1100-1900 PWM'e çeviriyoruz
            def map_pwm(val):
                return int(np.clip(1500 + (val * 400), 1100, 1900))

            ch1 = map_pwm(vpitch) # PITCH (Yunuslama)
            ch2 = map_pwm(0.0)    # ROLL  (Yatış - her zaman sabit 0)
            ch3 = map_pwm(vz)     # HEAVE (Yukarı/Aşağı / Throttle)
            ch4 = map_pwm(vyaw)   # YAW   (Kendi ekseninde dönüş)
            ch5 = map_pwm(vx)     # SURGE (İleri/Geri)
            ch6 = map_pwm(vy)     # SWAY  (Sağ/Sol)

            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                ch1, ch2, ch3, ch4, ch5, ch6, 0, 0
            )

    # ── ANA DÖNGÜ ─────────────────────────────────────────────────────────────

    def main_loop(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        self._update_parameters()
        deadzone_px    = self.get_parameter('deadzone_px').value
        dist_touchdown = self.get_parameter('dist_touchdown').value
        vz_search      = self.get_parameter('vz_search').value
        vz_dock_far    = self.get_parameter('vz_dock_far').value
        vz_dock_near   = self.get_parameter('vz_dock_near').value
        lost_timeout   = self.get_parameter('lost_timeout').value
        
        # Otonom aktifken uygulanacak pitch açısı
        pitch_tilt     = self.get_parameter('pitch_tilt').value 

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        img_pts, obj_pts = [], []
        self.num_detected = 0

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                if m_id in TARGET_IDS:
                    self.num_detected += 1
                    cx_3d, cy_3d, _ = TARGET_3D_POINTS[m_id]
                    
                    marker_3d_corners = [
                        [cx_3d - half_size, cy_3d - half_size, 0.0],
                        [cx_3d + half_size, cy_3d - half_size, 0.0],
                        [cx_3d + half_size, cy_3d + half_size, 0.0],
                        [cx_3d - half_size, cy_3d + half_size, 0.0]
                    ]
                    obj_pts.extend(marker_3d_corners)
                    img_pts.extend(corners[i][0].tolist())

        if self.num_detected > 0:
            success, rvec, tvec = cv2.solvePnP(
                np.array(obj_pts, dtype=np.float32),
                np.array(img_pts, dtype=np.float32),
                CAMERA_MATRIX,
                DIST_COEFFS,
                flags=cv2.SOLVEPNP_SQPNP
            )
            
            if success and tvec[2][0] > 0:
                center_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
                mx = center_2d[0][0][0]
                my = center_2d[0][0][1]

                self.last_cx = int(self.filter_cx.update(mx))
                self.last_cy = int(self.filter_cy.update(my))
                self.current_dist = self.filter_dist.update(float(tvec[2][0]))
                self.locked = True
                self.last_locked_time = time.time()
        else:
            self.locked = False

        # ── KONTROL MANTIĞI ──
        if self.master and not self.armed:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0
            )
            if self.master.motors_armed():
                self.armed = True
            self.state = ROVState.ARMING

        vx, vy, vz, vyaw, vpitch = 0.0, 0.0, 0.0, 0.0, 0.0

        if self.locked:
            self.state = ROVState.DOCKING
            
            # Araç aşağı baktığı için, ekranın Y ekseni (yukarı-aşağı) ileri gitme (Surge) hareketine denk düşer
            err_x = self.last_cx - (IMAGE_WIDTH  // 2)
            err_y = self.last_cy - (IMAGE_HEIGHT // 2)

            if abs(err_x) < deadzone_px: err_x = 0
            if abs(err_y) < deadzone_px: err_y = 0

            vy = -self.pid_sway.update(err_x)
            vx = -self.pid_surge.update(err_y)
            vpitch = pitch_tilt  # Aracın burnunu eğik tutmaya devam et

            if self.current_dist <= dist_touchdown:
                self.state = ROVState.TOUCHDOWN
                vz = vz_dock_near
            else:
                vz = vz_dock_far
        else:
            if self.armed:
                time_since_lost = time.time() - self.last_locked_time
                if time_since_lost < lost_timeout:
                    self.state = ROVState.ALIGNING
                    vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
                    vpitch = pitch_tilt 
                else:
                    self.state = ROVState.SEARCHING
                    vz = vz_search
                    vpitch = pitch_tilt # Arama esnasında da aşağıya bakarak ilerlesin

        # MAVLink üzerinden 6 eksenli Override'ı gönder
        self.send_rc_override(vx, vy, vz, vyaw, vpitch)

        self.draw_hud(frame, deadzone_px)
        try:
            self.pub_processed_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except: pass

    # ── HUD ───────────────────────────────────────────────────────────────────

    def draw_hud(self, frame, deadzone_px):
        color      = (0, 255, 0) if self.locked else (0, 0, 255)
        deadzone_px = int(deadzone_px)
        cv2.drawMarker(frame, (self.last_cx, self.last_cy), color, cv2.MARKER_CROSS, 30, 2)
        cv2.circle(frame, (IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2), deadzone_px, (255, 255, 0), 1)
        
        # Pitch durumu HUD'a eklendi
        pitch_val = self.get_parameter('pitch_tilt').value
        cv2.putText(frame,
            f"MODE: {self.state.name} | DIST: {self.current_dist:.2f}m | TILT: {pitch_val:.2f}",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2
        )

    # ── KAPAT ─────────────────────────────────────────────────────────────────

    def shutdown(self):
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            # Motorları boşta bırak (1500 nötr PWM gönder)
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        if self.cap:
            self.cap.release()


def main():
    rclpy.init()
    node = DownwardDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()