import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Point

# ── PARAMETRELER ──────────────────────────────────────────────────────────────
MARKER_SIZE      = 0.15  # 15x15 cm marker boyutu
IMAGE_WIDTH      = 1920
IMAGE_HEIGHT     = 1080
FOCAL_LENGTH_PX  = 1080.0 # güncelle

# Kamera matrisi ve bükülme katsayılarını kalibrasyonla günceelle
CAMERA_MATRIX = np.array([[FOCAL_LENGTH_PX, 0, 960], [0, FOCAL_LENGTH_PX, 540], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS   = np.array([-0.03, -0.01, 0, 0], dtype=np.float32) 

# Doklanma Eşikleri
DIST_APPROACH      = 2.0  # Yavaşlamanın başladığı mesafe (m)
DIST_TOUCHDOWN     = 0.45 # Dikey çöküşün başladığı mesafe
DEADZONE_PX        = 15  # 15 piksellik hata payı (titrememesi için)

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
        # np.clip integralin sonsuza gitmesini engeller
        self.integral = np.clip(self.integral + error * dt, -self.limit, self.limit)
        derivative = (error - self.last_error) / dt
        self.last_error, self.last_time = error, time.time()
        return np.clip((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative), -self.limit, self.limit)

# ── ANA DÜĞÜM ─────────────────────────────────────────────────────────────────
class DownwardDockingNode(Node):
    def __init__(self):
        super().__init__('downward_docking_node')
        
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pub_target_center = self.create_publisher(Point, '/aruco/target_center', 10)

        # PlotJuggler Debug Yayıncıları
        self.pub_err_x = self.create_publisher(Float32, '/debug/err_x', 10)
        self.pub_err_y = self.create_publisher(Float32, '/debug/err_y', 10)
        
        self.bridge = CvBridge()
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL))
        
        self.locked = False
        self.current_dist = 2.0
        self.last_cx, self.last_cy = 960, 540
        self.last_img_time = time.time()
        self.num_detected = 0 # HUD için sınıf değişkeni yapıldı
        self.mode_name = "WAITING"

        # PID Kontrolörler (AŞAĞI BAKIŞ - 8 THRUSTER)
        self.pid_surge = PID(0.0014, 0.0001, 0.0005, 0.6) # Y Hatası -> Surge
        self.pid_sway  = PID(0.0014, 0.0001, 0.0005, 0.6) # X Hatası -> Sway
        self.pid_heave = PID(0.4, 0.01, 0.1, 0.5)         # Mesafe -> Heave
        self.pid_yaw   = PID(0.5, 0.001, 0.05, 0.4)       # Açısal Hata -> Yaw

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Aşağı Bakışlı İniş Sistemi Aktif!")

    def image_callback(self, msg):
        self.last_img_time = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
        corners, ids, _ = self.detector.detectMarkers(clahe)
        
        self.locked = False
        self.num_detected = 0 if ids is None else len(ids)
        mx, my = None, None

        if ids is not None:
            pts = [np.mean(c[0], axis=0) for c in corners]

            # --- DURUM 1: 4 MARKER (Full Ortalama) ---
            if self.num_detected >= 4:
                all_pts = np.array(pts)
                mx, my = np.mean(all_pts[:, 0]), np.mean(all_pts[:, 1])
                self.locked = True

            # --- DURUM 2: 3 MARKER (Diyagonal Seçimi) ---
            elif self.num_detected == 3:
                max_d = 0
                p1, p2 = pts[0], pts[1]
                for i in range(3):
                    for j in range(i+1, 3):
                        d = np.linalg.norm(pts[i] - pts[j])
                        if d > max_d: max_d, p1, p2 = d, pts[i], pts[j]
                mx, my = (p1 + p2) / 2
                self.locked = True

            # --- DURUM 3: 2 MARKER (Sadece Diyagonal ise) ---
            elif self.num_detected == 2:
                dist_px = np.linalg.norm(pts[0] - pts[1])
                # Diyagonal yaklaşık 1.41 * kenar ama dikdörtgen için 0.89m baz alınabilir
                expected_diag_px = (0.89 * FOCAL_LENGTH_PX) / max(self.current_dist, 0.1)
                
                if dist_px > (expected_diag_px * 0.70): # %70 diyagonal eşiği
                    mx, my = (pts[0] + pts[1]) / 2
                    self.locked = True

            # Mesafe her durumda güncellenir (ilk marker üzerinden)
            success, _, tvec = cv2.solvePnP(np.array([[-0.075,0.075,0],[0.075,0.075,0],[0.075,-0.075,0],[-0.075,-0.075,0]], dtype=np.float32), 
                                     corners[0][0].astype(np.float32), CAMERA_MATRIX, DIST_COEFFS)
            if success:
                self.current_dist = float(np.linalg.norm(tvec))

            if self.locked:
                self.last_cx, self.last_cy = int(mx), int(my)

        # HUD her frame güncellenir
        self.render_hud(frame)
        cv2.imshow('Downward ROV View', cv2.resize(frame, (960, 540)))
        cv2.waitKey(1)

    def render_hud(self, frame):
        #marker tespit edildiyse yeşil edilmediyse kırmızı yazı
        color = (0, 255, 0) if self.locked else (0, 0, 255)
        cv2.putText(frame, f"DIST: {self.current_dist:.2f}m", (50, 50), 0, 1.2, (255, 255, 0), 2)
        if self.locked:
            #arucolara yeşil çarpı çiz
            cv2.drawMarker(frame, (self.last_cx, self.last_cy), (0, 255, 0), cv2.MARKER_CROSS, 40, 3)
            #ekranın tam ortasına mor daire çizer
            cv2.circle(frame, (960, 540), 15, (255, 0, 255), 2) # Kamera Merkezi
            # Hangi durumdayız yazdır
            cv2.putText(frame, f"Markers: {self.num_detected} Markers", (50, 150), 0, 1, (0, 255, 255), 2)
            cv2.putText(frame, f"MODE: {self.mode_name}", (50, 120), 0, 1.2, color, 3)
            cv2.putText(frame, f"REQ SURGE (vx): {self.last_vx:.2f}", (50, 200), 0, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"REQ SWAY  (vy): {self.last_vy:.2f}", (50, 230), 0, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"REQ HEAVE (vz): {self.last_vz:.2f}", (50, 260), 0, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"VYaw (Rot): {self.last_vyaw:.2f}", (50, 210), 0, 0.7, (0, 255, 255), 2)
    
    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_link'
        now = time.time()
        
        #kamera kesilirse araç durur
        if now - self.last_img_time > 3.0:
            self.pub_cmd.publish(cmd)
            return
        
        # Kilitlenme yoksa (Arama Modu): Yüksel
        if not self.locked:
            self.mode_name = "SEARCHING"
            cmd.twist.linear.z = 0.20  # Yavaşça yukarı itiş (Arama alanını genişletmek için)
            self.pub_cmd.publish(cmd)
            return
        
        # Hata Hesaplama
        err_x = self.last_cx - (IMAGE_WIDTH // 2) # Yanal hata
        err_y = self.last_cy - (IMAGE_HEIGHT // 2)  # İleri-Geri hata

        target_msg = Point()
        target_msg.x = float(self.last_cx)
        target_msg.y = float(self.last_cy)
        target_msg.z = float(self.current_dist) # İsteğe bağlı: Z eksenine mesafeyi koyabilirsin

        self.pub_target_center.publish(target_msg)

        if abs(err_x) < DEADZONE_PX: err_x = 0 # hata çok küçükse tepki vermez
        if abs(err_y) < DEADZONE_PX: err_y = 0

        # PID Çıktıları
        vy = -self.pid_sway.update(err_x)
        vx = -self.pid_surge.update(err_y)
        vyaw = -self.pid_yaw.update(self.current_yaw_err)
        self.get_logger().info(
            f"HATA -> X:{err_x:4d} Y:{err_y:4d} | "
            f"GEREKEN -> Surge(vx):{vx:.2f} Sway(vy):{vy:.2f} Heave(vz):{vz:.2f} Yaw(vyaw):{vyaw:.2f}",
            throttle_duration_sec=0.5 # Terminali kirletmemesi için yarım saniyede bir yazdırır
        )
        # İniş Mantığı
        if self.current_dist > DIST_TOUCHDOWN:
            self.mode_name = "APPROACHING"
            vz = -0.15 # Yavaş alçalma
        else:
            self.mode_name = "TOUCHDOWN"
            vx, vy = vx * 0.4, vy * 0.4 # Dokunma anında yanal hareketleri daha çok kıs
            vz = -0.10 # Yumuşak iniş

        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z, cmd.twist.angular.z = float(vx), float(vy), float(vz), float(vyaw)
        self.pub_cmd.publish(cmd)
        
        # PlotJuggler için yayınla
        self.pub_err_x.publish(Float32(data=float(err_x)))
        self.pub_err_y.publish(Float32(data=float(err_y)))

def main():
    rclpy.init()
    node = DownwardDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sistem kullanıcı tarafından kapatıldı.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()