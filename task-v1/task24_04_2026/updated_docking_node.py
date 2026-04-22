import os
import time
import rclpy
from rclpy.node import Node

os.environ['MAVLINK20'] = '1'

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pymavlink import mavutil

from config import (
    IMAGE_WIDTH, IMAGE_HEIGHT,
    MAX_LATERAL, MAX_YAW,
    DIST_ALIGN, DIST_DESCENDING, FINAL_ALIGN_DIST, DIST_DOCKED,
    PRECISION_THRESH, HOVER_TIME, FAILSAFE_LOST,
    SERIAL_PORT, BAUD_RATE
)
from controller import PID, Kalman2D
from vision import Vision

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        self.declare_parameter('video_source', 0)
        self.video_source = self.get_parameter('video_source').value

        self.cap = cv2.VideoCapture(self.video_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, '/docking/camera_view', 10)

        self.vision = Vision()
        self.kalman = Kalman2D()

        # Sallanmayı önlemek için güncellenmiş PID katsayıları
        self.pid_x = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_y = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_yaw = PID(0.5, 0.02, 0.3, MAX_YAW)

        self.state = "SEARCHING"
        self.hover_start = None
        self.blind_start_t = None
        self.is_settled = False
        self.confirmed_4_markers = False
        self.approach_confidence = 0
        self.search_start_t = None

        self.last_seen_t = time.time()
        self.armed = False

        # sonradan eklendi
        # Yumuşak Kilit için son komutları tutan değişkenler
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0

        self.master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        self.master.wait_heartbeat(timeout=5)
        self.get_logger().info("✅ Pixhawk Bağlantısı OK")

        self._set_mode("MANUAL")
        self.create_timer(0.04, self.loop)

    def _set_mode(self, mode):
        if self.master:
            try:
                mode_id = self.master.mode_mapping()[mode]
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                self.get_logger().info(f"✅ Uçuş Modu {mode} olarak ayarlandı.")
            except Exception as e:
                self.get_logger().error(f"Mod ayarlanırken hata: {e}")

    def send(self, vx, vy, vz, yaw):
        if not self.master: return
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(vx*1000),
            int(vy*1000),
            int(500 + vz*1000),
            int(yaw*1000),
            0
        )

    def loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame, visible, locked, n_valid, target_tvec, target_yaw, d_v = self.vision.process(frame)

        x_w, y_w, d_h = 0.0, 0.0, 0.0

        # ── Kontrol Hesaplama ──
        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
        vz_log = 0.0 # Terminal logu için

        if visible and target_tvec is not None:
            self.last_seen_t = time.time()
            raw_x, raw_y = self.vision.compute_world(target_tvec)
            x_w, y_w = self.kalman.update([raw_x, raw_y])
            d_h = np.sqrt(x_w**2 + y_w**2)

            # PID çıktılarını hafızaya al
            self.last_vx = self.pid_x.update(y_w)
            self.last_vy = self.pid_y.update(x_w)
            self.last_vyaw = self.pid_yaw.update(-target_yaw)

        # ── Terminale Detaylı Bilgi Basma (Her 10 döngüde bir - Terminali kirletmemek için) ──
        if int(time.time() * 25) % 10 == 0:
            self.get_logger().info(
                f"[{self.state}] Vis:{visible} | MKR:{n_valid} | "
                f"x:{x_w:.2f} y:{y_w:.2f} dv:{d_v:.2f} | "
                f"vx:{self.last_vx:.2f} vy:{self.last_vy:.2f} vz:{vz_log:.2f}"
            )

        if n_valid >= 4:
            self.approach_confidence += 1
            if self.approach_confidence >= CONFIDENCE_THRESHOLD:
                self.confirmed_4_markers = True
                self.get_logger().info("✅ 4 Marker Onaylandı. İniş yetkisi verildi.")
        else:
            self.approach_confidence = max(0, self.approach_confidence - 1)

        # ── FSM Durum Geçişleri ──
        # (FSM blokların burada aynen kalıyor, sadece logları info seviyesinde tut)
        if self.state == "SEARCHING":
            # Arama modundaysak hafızayı temizle
            self.confirmed_4_markers = False
            self.approach_confidence = 0
            
            if self.search_start_t is None:
                self.search_start_t = time.time()

            elapsed = time.time() - self.search_start_t

            # Sadece hedef 10 saniyeden uzun süredir kayıpsa onayları sıfırla
            # Bu sayede anlık marker kayıplarında 'confirmed_4_markers' True kalmaya devam eder
            if elapsed > 10.0:
                self.confirmed_4_markers = False
                self.approach_confidence = 0
                self.get_logger().info(">>> Hedef 10s boyunca bulunamadı, hafıza sıfırlandı.")

            # marker görüldüyse aligbinge geç, yoksa 5 saniye ileri sonra dönmeye başla
            if visible:
                self.state = "ALIGNING"
                self.search_start_t = None # Zamanlayıcıyı sıfırla
                self.get_logger().info(">>> SEARCHING -> ALIGNING")
            else:
                elapsed = time.time() - self.search_start_t
                
                if elapsed < 5.0: # İlk 5 saniye ileri git
                    vx = 0.15    # %15 hızla ileri
                    vyaw = 0.0
                    self.get_logger().info("Searching: İleri gidiliyor...", once=True)
                else:            # 5 saniye dolunca dönmeye başla
                    vx = 0.0
                    vyaw = 0.2   # Kendi ekseninde tarama hızı
                    self.get_logger().info("Searching: Etraf taranıyor...", once=True)
        
        elif self.state == "ALIGNING" and d_h < DIST_ALIGN:
            if y_w < 1.5: # Belirli bir yakınlığa gelince yaklaşmaya başla
                self.state = "APPROACHING"
            self.get_logger().info(">>> ALIGNING -> APPROACHING")
        
        elif self.state == "APPROACHING" and y_w > DIST_DESCENDING:
            
            # Sadece ofset değerini geçmesi değil, ofsetin +/- 15cm yakınında olması şartı
            upper_bound = CAMERA_OFFSET_Y + 0.15
            lower_bound = CAMERA_OFFSET_Y - 0.15
            
            # KRİTİK ŞART: 4 marker görüldü mü VE araç merkezi ofset değerini geçti mi?
            if self.confirmed_4_markers and y_w < CAMERA_OFFSET_Y:
                self.state = "DESCENDING"
                self.get_logger().warn("!!! DESCENDING STARTED")
        
        elif self.state == "DESCENDING" and d_v < FINAL_ALIGN_DIST:
            self.state = "FINAL_ALIGN"
            self.get_logger().info(">>> DESCENDING -> FINAL_ALIGN")
        
        elif (self.state == "FINAL_ALIGN" or self.state == "DESCENDING") and d_v < DIST_DOCKED:
            self.state = "DOCKED"
            self.get_logger().info("✅ DOCKED: Görev Tamamlandı.")

        
        if self.state == "DOCKED":
            pass
        else:
            if self.state == "DESCENDING": vz = -0.15
            elif self.state == "FINAL_ALIGN": vz = -0.08
            
            if visible:
                # İniş sırasında ileri gitmeyi durdurma mantığı korunuyor
                if self.state in ["DESCENDING", "FINAL_ALIGN"]:
                    vx, vy = 0.0, 0.0
                else:
                    vx, vy = self.last_vx, self.last_vy
                vyaw = self.last_vyaw
            else:
                # Görünürlük yoksa sönümleme (Searching hızı burayı ezmez çünkü o üstte)
                if self.state in ["DESCENDING", "FINAL_ALIGN", "APPROACHING", "ALIGNING"]:
                    vx, vy, vyaw = self.last_vx * 0.8, self.last_vy * 0.8, self.last_vyaw * 0.8
            
        vz_log = vz
        self.send(vx, vy, vz, vyaw)

        # ── EKRAN (HUD) ÜZERİNE DETAYLI BİLGİ YAZDIRMA ──
        h_color = (0, 255, 0) if visible else (0, 0, 255)
        # 1. Satır: Durum ve Görünürlük
        cv2.putText(frame, f"STATE: {self.state}", (10, 30), 2, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"VISIBLE: {visible} ({n_valid} Mkr)", (10, 55), 2, 0.6, h_color, 2)
        
        # 2. Satır: Koordinat Hataları (Kalman Filtreli)
        cv2.putText(frame, f"X_Err: {x_w:.2f}m", (10, 90), 2, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Y_Err: {y_w:.2f}m", (10, 110), 2, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Z_Dist: {d_v:.2f}m", (10, 130), 2, 0.5, (0, 255, 255), 1)

        # 3. Satır: Aktif Motor Komutları (vx, vy, vz, vyaw)
        cv2.putText(frame, "COMMANDS:", (450, 30), 2, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, f"VX: {vx:.2f}", (450, 50), 2, 0.5, (100, 255, 100), 1)
        cv2.putText(frame, f"VY: {vy:.2f}", (450, 70), 2, 0.5, (100, 255, 100), 1)
        cv2.putText(frame, f"VZ: {vz:.2f}", (450, 90), 2, 0.5, (100, 255, 100), 1)
        cv2.putText(frame, f"YW: {vyaw:.2f}", (450, 110), 2, 0.5, (100, 255, 100), 1)

        # 4. Satır: Hedef Sınırlar (Thresholds)
        cv2.putText(frame, f"Desc_Limit: {DIST_DESCENDING}m", (10, 450), 2, 0.5, (200, 200, 200), 1)
        
        # Görüntüyü Yayınla
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_img.publish(img_msg)
        except: pass

def main():
    rclpy.init()
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()