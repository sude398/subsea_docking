import os
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Point
from pymavlink import mavutil

# MAVLink 2.0 protokolünü zorla
os.environ['MAVLINK20'] = '1'

# ── PARAMETRELER ──────────────────────────────────────────────────────────────
MARKER_SIZE      = 0.15 
IMAGE_WIDTH      = 1920
IMAGE_HEIGHT     = 1080
FOCAL_LENGTH_PX  = 1080.0
CAMERA_MATRIX    = np.array([[FOCAL_LENGTH_PX, 0, 960], [0, FOCAL_LENGTH_PX, 540], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS      = np.array([-0.03, -0.01, 0, 0], dtype=np.float32)

DIST_TOUCHDOWN   = 0.45
DEADZONE_PX      = 15

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200

RC_MIN = -1000
RC_MAX =  1000
RC_MID =  500 

# ── PID SINIFI ────────────────────────────────────────────────────────────────
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
        return np.clip((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative), -self.limit, self.limit)

# ── ANA DÜĞÜM ─────────────────────────────────────────────────────────────────
class DownwardDockingNode(Node):
    def __init__(self):
        super().__init__('downward_docking_node')

        # Abonelikler
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Yayıncılar
        self.pub_target_center = self.create_publisher(Point, '/aruco/target_center', 10)
        # Görüntü Yayıncısı (RViz veya Web GUI için)
        self.pub_processed_img = self.create_publisher(Image, '/aruco/image_processed', 10)
        
        self.bridge = CvBridge()
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL))

        # Durum Değişkenleri
        self.locked = False
        self.current_dist = 2.0
        self.last_cx, self.last_cy = 960, 540
        self.last_img_time = time.time()
        self.mode_name = "CONNECTING"
        self.armed = False
        self.num_detected = 0

        # PID Kontrolörler (Yatay eksende hassas tutuş)
        self.pid_surge = PID(0.0012, 0.0001, 0.0004, 0.5)
        self.pid_sway  = PID(0.0012, 0.0001, 0.0004, 0.5)

        # MAVLink
        self.master = None
        self._connect_pixhawk()

        # Heartbeat Thread
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb_thread.start()

        # Kontrol döngüsü (20 Hz)
        self.create_timer(0.05, self.control_loop)

    def _connect_pixhawk(self):
        self.get_logger().info(f"🔌 Pixhawk'a bağlanılıyor...")
        try:
            self.master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, source_system=255)
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info("✅ Pixhawk Bağlantısı OK!")
            self._set_param("ARMING_CHECK", 0) 
            self._set_param("FS_GCS_EN", 0)
            self._set_mode("MANUAL")
        except Exception as e:
            self.get_logger().error(f"Bağlantı hatası: {e}")

    def _set_param(self, name, value):
        if self.master:
            self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                          name.encode('utf-8'), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def _set_mode(self, mode):
        if self.master:
            try:
                mode_id = self.master.mode_mapping()[mode]
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            except: pass

    def _heartbeat_loop(self):
        while rclpy.ok():
            if self.master:
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1.0)

    def send_manual_control(self, vx, vy, vz, vyaw):
        if self.master:
            rx = int(np.clip(vx * 1000, -1000, 1000))
            ry = int(np.clip(vy * 1000, -1000, 1000))
            rz = int(np.clip(RC_MID + (vz * 1000), 0, 1000))
            rr = int(np.clip(vyaw * 1000, -1000, 1000))
            self.master.mav.manual_control_send(self.master.target_system, rx, ry, rz, rr, 0)

    def image_callback(self, msg):
        self.last_img_time = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ArUco Tespit
        corners, ids, _ = self.detector.detectMarkers(gray)

        self.locked = False
        self.num_detected = 0 if ids is None else len(ids)

        if ids is not None:
            # İşaretçileri kare üzerine çiz
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Merkez hesapla
            all_pts = np.array([np.mean(c[0], axis=0) for c in corners])
            mx, my = np.mean(all_pts[:, 0]), np.mean(all_pts[:, 1])
            self.last_cx, self.last_cy = int(mx), int(my)
            self.locked = True
            
            # Mesafe tahmini
            success, _, tvec = cv2.solvePnP(
                np.array([[-0.075, 0.075, 0], [0.075, 0.075, 0], [0.075, -0.075, 0], [-0.075, -0.075, 0]], dtype=np.float32),
                corners[0][0].astype(np.float32), CAMERA_MATRIX, DIST_COEFFS)
            if success: self.current_dist = float(np.linalg.norm(tvec))

        # HUD bilgilerini çiz
        self.draw_hud(frame)

        # GÖRÜNTÜYÜ YAYINLA
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_msg.header = msg.header # Zaman damgasını koru
            self.pub_processed_img.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Görüntü yayınlanamadı: {e}")

        # Local izleme (isteğe bağlı, headless sistemlerde kapatılabilir)
        # cv2.imshow('ROV Processed View', cv2.resize(frame, (640, 480)))
        # cv2.waitKey(1)

    def draw_hud(self, frame):
        """Kare üzerine görsel bilgiler ekler."""
        status_color = (0, 255, 0) if self.locked else (0, 0, 255)
        # Hedef Crosshair
        cv2.drawMarker(frame, (self.last_cx, self.last_cy), status_color, cv2.MARKER_CROSS, 30, 2)
        # Ekran Merkezi
        cv2.circle(frame, (IMAGE_WIDTH//2, IMAGE_HEIGHT//2), DEADZONE_PX, (255, 255, 0), 1)
        
        # Metin Bilgileri
        cv2.putText(frame, f"MODE: {self.mode_name}", (50, 50), 0, 1, status_color, 2)
        cv2.putText(frame, f"DIST: {self.current_dist:.2f}m", (50, 90), 0, 1, (255, 255, 0), 2)
        cv2.putText(frame, f"MARKERS: {self.num_detected}", (50, 130), 0, 1, (255, 255, 0), 2)

    def control_loop(self):
        if self.master and not self.armed:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            if self.master.motors_armed(): self.armed = True
            return

        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
        if self.locked:
            self.mode_name = "DOCKING"
            err_x = self.last_cx - (IMAGE_WIDTH // 2)
            err_y = self.last_cy - (IMAGE_HEIGHT // 2)
            if abs(err_x) < DEADZONE_PX: err_x = 0
            if abs(err_y) < DEADZONE_PX: err_y = 0
            vy = -self.pid_sway.update(err_x)
            vx = -self.pid_surge.update(err_y)
            vz = -0.15 if self.current_dist > DIST_TOUCHDOWN else -0.07
        else:
            self.mode_name = "SEARCHING"
            vz = 0.12 # Kaybederse hafif yükselerek alanı tara
            
        self.send_manual_control(vx, vy, vz, vyaw)

def main():
    rclpy.init()
    node = DownwardDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()