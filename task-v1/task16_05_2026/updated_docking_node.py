import os
import time
import math

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
    PRECISION_THRESH, HOVER_TIME, FAILSAFE_LOST, CONFIDENCE_THRESHOLD,
    CAMERA_OFFSET_Y, SERIAL_PORT, BAUD_RATE
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
        self.approach_start_t = None
        self.local_search_start_t = None

        self.last_seen_t = time.time()
        self.last_loop_t = time.time()
        self.d_v = 999.0
        self.last_vz_cmd = 0.0
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
        now = time.time()
        dt = now - self.last_loop_t
        self.last_loop_t = now

        ret, frame = self.cap.read()
        if not ret: return

        frame, visible, locked, n_valid, target_tvec, target_yaw, vision_d_v = self.vision.process(frame)

        # HEDEF KAYMASI (TARGET SNAP) KORUMASI:
        # Araç 4 marker onayı aldıysa, artık hedefin köşede değil merkezde olduğunu biliyordur.
        # Bu saatten sonra kısıtlı görüş açısıyla (sadece 1 marker görerek) hedefin köşeye kaymasını engelle!
        # "locked == True", merkezin kesin olarak hesaplanabildiği (2 diyagonal, 3 veya 4 mkr) anları temsil eder.
        if self.confirmed_4_markers and visible and not locked:
            visible = False  # Köşeyi merkez zannetme, hedefi yoksay (Kör Uçuşa Geç)
            cv2.putText(frame, "IGNORING CORNER (SNAP PROTECTION)", (10, 150), 2, 0.6, (0, 0, 255), 2)

        # Başlangıç değerleri (d_h 999.0 ile "kör" yaklaşma hatası önlenir)
        x_w, y_w, y_error, d_h = 0.0, 0.0, 0.0, 999.0
        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        if visible and target_tvec is not None:
            self.last_seen_t = time.time()
            self.d_v = vision_d_v
            raw_x, raw_y = self.vision.compute_world(target_tvec)
            x_w, y_w = self.kalman.update([raw_x, raw_y])
            
            # Kamera öndeyse, hedef kameranın gerisinde (-Y) kalmalıdır.
            # Y ekseni hatası (hedef - (-ofset)) -> y_w + CAMERA_OFFSET_Y
            y_error = y_w + CAMERA_OFFSET_Y
            
            d_h = np.sqrt(x_w**2 + y_error**2)

            self.last_vx = self.pid_x.update(y_error)
            self.last_vy = self.pid_y.update(x_w)
            self.last_vyaw = self.pid_yaw.update(-target_yaw)
            
        elif self.state in ["DESCENDING", "FINAL_ALIGN"]:
            # Görüş koptuğunda (kör bölge) hıza dayalı tahmini yükseklik hesabı (Dead Reckoning)
            # vz eksi olduğu için d_v'yi azaltarak kalan mesafeyi tahmin eder
            self.d_v += self.last_vz_cmd * dt
            self.d_v = max(0.0, self.d_v)

        # ── Güven ve Onay Mekanizması ──
        if n_valid >= 4:
            self.approach_confidence = min(CONFIDENCE_THRESHOLD + 10, self.approach_confidence + 1)
        else:
            # İniş aşamasındaysak güven puanını düşürme (marker kaybı normaldir)
            if self.state not in ["DESCENDING", "FINAL_ALIGN"]:
                self.approach_confidence = max(0, self.approach_confidence - 1)

        # Onay Kilidi: Sadece Searching modunda sıfırlanır
        if self.approach_confidence >= CONFIDENCE_THRESHOLD and not self.confirmed_4_markers:
            self.confirmed_4_markers = True
            self.get_logger().info("✅ 4 Marker Onaylandı. İniş yetkisi verildi.")

        # ── FSM (Sonlu Durum Makinesi) ──
        if self.state == "SEARCHING":
            # Arama modunda hafızayı temizle
            if self.confirmed_4_markers:
                self.confirmed_4_markers = False
                self.get_logger().info(">>> SEARCHING: Yetkiler sıfırlandı.")
            
            self.approach_confidence = 0
            
            if self.search_start_t is None:
                self.search_start_t = time.time()

            if visible:
                self.state = "ALIGNING"
                self.search_start_t = None
                self.get_logger().info(">>> SEARCHING -> ALIGNING")
            else:
                elapsed = time.time() - self.search_start_t
                if elapsed < 5.0:
                    vx = 0.15
                    vyaw = 0.0
                else:
                    vx = 0.0
                    vyaw = 0.2
        
        elif self.state == "ALIGNING":
            if not visible and (time.time() - self.last_seen_t > 3.0):
                self.state = "LOCAL_SEARCH"
                self.local_search_start_t = time.time()
                self.get_logger().warn(">>> Hedef Kayboldu: ALIGNING -> LOCAL_SEARCH")
            elif d_h < DIST_ALIGN:                
                self.state = "APPROACHING"
                self.approach_start_t = time.time()
                self.get_logger().info(">>> ALIGNING -> APPROACHING")
        
        elif self.state == "APPROACHING":
            if self.confirmed_4_markers and d_h < DIST_DESCENDING:
                # İNİŞ İZNİ: 4 marker önceden onaylandı, yatayda merkeze varıldı (kör bile olsa in)
                self.state = "DESCENDING"
                self.get_logger().warn("!!! DESCENDING STARTED")
            elif self.confirmed_4_markers and not visible and (time.time() - self.last_seen_t > 5.0):
                # GÜVENLİK KİLİDİ: Onaylı hedef 5 saniyedir tamamen kayıpsa (ör. araya balık girdiyse),
                # havada sonsuza dek asılı kalmamak için onayı iptal et ve sarmal aramaya çık.
                self.confirmed_4_markers = False
                self.state = "LOCAL_SEARCH"
                self.local_search_start_t = time.time()
                self.get_logger().error(">>> ONAYLI HEDEF KAYBEDİLDİ: APPROACHING -> LOCAL_SEARCH")
            elif not self.confirmed_4_markers:
                # Henüz onay alınmadıysa güvenlik kuralları geçerlidir
                if not visible and (time.time() - self.last_seen_t > 3.0):
                    self.state = "LOCAL_SEARCH"
                    self.local_search_start_t = time.time()
                    self.get_logger().warn(">>> Hedef Kayboldu: APPROACHING -> LOCAL_SEARCH")
                elif self.approach_start_t and (time.time() - self.approach_start_t) > 4.0:
                    self.state = "LOCAL_SEARCH"
                    self.local_search_start_t = time.time()
                    self.get_logger().info(">>> APPROACHING -> LOCAL_SEARCH: SARMAL ARAMA BASLADI")
                    
        elif self.state == "LOCAL_SEARCH":
            if not visible and (time.time() - self.last_seen_t > 4.0):
                self.state = "SEARCHING"
                self.get_logger().warn(">>> Hedef Kayboldu: LOCAL_SEARCH -> SEARCHING")
            elif locked:
                # Gerçek merkez (2 diyagonal, 3 veya 4 marker) bulunduysa hedefe geri kilitlen
                self.state = "APPROACHING"
                self.approach_start_t = time.time() # kronometreyi sıfırla
                self.get_logger().info(">>> LOCAL_SEARCH -> APPROACHING: Gerçek Merkez Bulundu")
            else:
                vz = 0.15 # Yüksel (FOV genişlet)
                t_search = time.time() - self.local_search_start_t
                # Sarmal hız (dairesel hareket)
                vx = 0.2 * math.cos(t_search)
                vy = 0.2 * math.sin(t_search)
                cv2.putText(frame, "LOCAL SEARCH (SPIRAL)...", (10, 200), 2, 0.6, (0, 165, 255), 2)
        
        elif self.state == "DESCENDING":
            vz = -0.15
            if self.d_v < FINAL_ALIGN_DIST:
                self.state = "FINAL_ALIGN"
                self.get_logger().info(">>> DESCENDING -> FINAL_ALIGN")
        
        elif self.state == "FINAL_ALIGN":
            vz = -0.08
            if self.d_v < DIST_DOCKED:
                self.state = "DOCKED"
                self.get_logger().info("✅ DOCKED: Görev Tamamlandı.")
        
        elif self.state == "DOCKED":
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        # ── Komut Gönderimi ve Sönümleme (Damping) ──
        if self.state != "DOCKED":
            if self.state == "LOCAL_SEARCH":
                # FSM bloğunda hesaplanan sarmal vx, vy, vz komutları geçerlidir
                # Yönelimi (Yaw) sabit tut
                vyaw = self.last_vyaw
            elif self.state == "SEARCHING":
                # FSM bloğunda atanan arama hızları geçerlidir
                pass
            else:
                # ALIGNING, APPROACHING, DESCENDING, FINAL_ALIGN modları
                if visible:
                    # Hedef görülüyorsa PID komutlarını kullan
                    vx, vy = self.last_vx, self.last_vy
                    vyaw = self.last_vyaw
                else:
                    # Hedef kayıpsa Gerçek Sönümleme (Damping) yap
                    # Her döngüde hızı %20 azaltarak sıfıra yaklaştır
                    self.last_vx *= 0.8
                    self.last_vy *= 0.8
                    self.last_vyaw *= 0.8
                    vx, vy, vyaw = self.last_vx, self.last_vy, self.last_vyaw
            
            self.last_vz_cmd = vz
            self.send(vx, vy, vz, vyaw)

        # Terminal Logu (Her 0.5 saniyede bir yazdırır)
        if not hasattr(self, 'last_log_t'):
            self.last_log_t = 0
        if time.time() - self.last_log_t > 0.5:
            self.last_log_t = time.time()
            self.get_logger().info(
                f"[{self.state}] MKR:{n_valid} | "
                f"x_w:{x_w:.2f} y_w:{y_w:.2f} | "
                f"y_err(Ofsetli):{y_error:.2f} d_h:{d_h:.2f} d_v:{self.d_v:.2f} | "
                f"v_xyz:[{vx:.2f}, {vy:.2f}, {vz:.2f}]"
            )

        # ── EKRAN (HUD) ÜZERİNE DETAYLI BİLGİ YAZDIRMA ──
        h_color = (0, 255, 0) if visible else (0, 0, 255)
        # 1. Satır: Durum ve Görünürlük
        cv2.putText(frame, f"STATE: {self.state}", (10, 30), 2, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"VISIBLE: {visible} ({n_valid} Mkr)", (10, 55), 2, 0.6, h_color, 2)
        
        # 2. Satır: Koordinat Hataları (Kalman Filtreli ve Ofsetli)
        cv2.putText(frame, f"X_Err: {x_w:.2f}m", (10, 90), 2, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Y_Err(Offset): {y_error:.2f}m", (10, 110), 2, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Z_Dist: {self.d_v:.2f}m", (10, 130), 2, 0.5, (0, 255, 255), 1)

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