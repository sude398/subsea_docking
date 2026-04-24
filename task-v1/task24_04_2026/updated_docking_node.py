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
    PRECISION_THRESH, HOVER_TIME, FAILSAFE_LOST, CAMERA_OFFSET_Y, CONFIDENCE_THRESHOLD,
    BLIND_DESCENT_DIST, BLIND_DESCENT_TIMEOUT,
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

        # Kör iniş için son bilinen hedef pozu (FOV kaybında kullanılır)
        self.last_x_w = 0.0
        self.last_y_w = 0.0
        self.last_d_h = 999.0
        self.last_d_v = 999.0

        # d_v gürültüsüne karşı histerezis/kararlılık sayaçları
        self.desc_stable_cnt = 0
        self.final_stable_cnt = 0

        # vz komut yumuşatması: FINAL_ALIGN -> DOCKED gibi geçişlerde throttle step'ini
        # (ve buoyancy kaynaklı ani yukarı sıçramayı) engelleyen 1. derece low-pass
        self.last_vz_cmd = 0.0

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

        # Başlangıç değerleri (d_h 999.0 ile "kör" yaklaşma hatası önlenir)
        x_w, y_w, d_h = 0.0, 0.0, 999.0
        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        if visible and target_tvec is not None:
            self.last_seen_t = time.time()
            raw_x, raw_y = self.vision.compute_world(target_tvec)
            x_w, y_w = self.kalman.update([raw_x, raw_y])
            d_h = np.sqrt(x_w**2 + y_w**2)

            # Son bilinen pozu sakla (kör iniş sırasında kullanılır)
            self.last_x_w, self.last_y_w = x_w, y_w
            self.last_d_h, self.last_d_v = d_h, d_v

            # Hedefe yakın micro-salınımı bastırmak için PID girişlerine deadband
            # (PID sınıfına dokunulmadan, çağrı tarafında). 3 cm / 3° altındaki hatalar
            # sıfır kabul edilir → kp*e ve kd*d yok → motor buzz biter.
            APPROACH_ERR_DB = 0.03
            YAW_ERR_DB = 0.05
            dx_in = 0.0 if abs(x_w) < APPROACH_ERR_DB else x_w
            dy_in = 0.0 if abs(y_w) < APPROACH_ERR_DB else y_w
            dyaw_in = 0.0 if abs(target_yaw) < YAW_ERR_DB else -target_yaw

            self.last_vx = self.pid_x.update(dy_in)
            self.last_vy = self.pid_y.update(dx_in)
            self.last_vyaw = self.pid_yaw.update(dyaw_in)
        else:
            # Kör mod: FSM kontrolleri sıfıra düşmesin diye son bilinen pozu kullan
            x_w, y_w = self.last_x_w, self.last_y_w
            d_h, d_v = self.last_d_h, self.last_d_v

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
                # Son görülmeden bu yana ≤15 s geçtiyse ve kayda değer bir lateral
                # ofset bilgisi varsa, önce o yöne dönerek "last seen" ipucunu kullan.
                recent_lost = (time.time() - self.last_seen_t) < 15.0
                use_hint = recent_lost and abs(self.last_x_w) > 0.05 and elapsed < 3.0

                if use_hint:
                    # Son görülen yöne yaw; çok hafif ileri hareket
                    # last_x_w > 0 → hedef sağda → vyaw pozitif (sağa dön)
                    vx = 0.05
                    vyaw = 0.25 * float(np.sign(self.last_x_w))
                else:
                    # 12 s'lik döngüsel keşif. Her döngüde 3 faz:
                    #   0-4 s : spiral (ileri + kademeli artan yaw)
                    #   4-8 s : + yönde sweep (dönerek tara, yerinde)
                    #   8-12 s: − yönde sweep (simetrik, kör noktaları kapatır)
                    # Döngü periyodik tekrar eder → takılıp kalmayı ve dead-zone'ları kırar.
                    base_t = elapsed - 3.0 if recent_lost else elapsed
                    cycle_t = base_t % 12.0 if base_t > 0 else 0.0

                    if cycle_t < 4.0:
                        # Spiral: sabit vx + yaw 0.08 → 0.25 arası lineer artar
                        vx = 0.12
                        vyaw = 0.08 + 0.17 * (cycle_t / 4.0)
                    elif cycle_t < 8.0:
                        vx = 0.0
                        vyaw = 0.25
                    else:
                        vx = 0.0
                        vyaw = -0.25
        
        elif self.state == "ALIGNING":
            if not visible and (time.time() - self.last_seen_t > 2.0):
                self.state = "SEARCHING"
                self.get_logger().warn(">>> Hedef Kayboldu: ALIGNING -> SEARCHING")
            elif d_h < DIST_ALIGN:                
                self.state = "APPROACHING"
                self.get_logger().info(">>> ALIGNING -> APPROACHING")
        
        elif self.state == "APPROACHING":
            # Yakın mesafede kamera FOV'u yüzünden marker kaybı beklenen bir durumdur.
            # Bu pencerede SEARCHING'e düşmeyi engelle (kör iniş).
            blind_active = (
                self.last_d_h < BLIND_DESCENT_DIST
                and (time.time() - self.last_seen_t) < BLIND_DESCENT_TIMEOUT
            )
            if not visible and (time.time() - self.last_seen_t > 3.0) and not blind_active:
                self.state = "SEARCHING"
                self.get_logger().warn(">>> Hedef Kayboldu: APPROACHING -> SEARCHING")
            elif self.confirmed_4_markers and y_w < CAMERA_OFFSET_Y:
                self.state = "DESCENDING"
                self.get_logger().warn("!!! DESCENDING STARTED (blind=%s)" % (not visible))
        
        elif self.state == "DESCENDING":
            # Mesafeye göre adaptif iniş hızı: uzakta agresif (-0.15), hedefe yaklaşırken
            # yumuşak (-0.05). Sabit vz'nin yarattığı throttle step'ini ve sonrasındaki
            # buoyancy kaynaklı ani yukarı çıkışı engeller.
            vz = max(-0.15, -0.05 - 0.1 * d_v)
            # Histerezis: d_v solvePnP çıktısı gürültülüdür. Tek frame'lik sivri inişler
            # vz'yi anında düşürüp görünür bir "sallanma/geri dönme" hissi yaratıyordu.
            # Eşikten 0.03 m aşağıda, 3 ardışık frame stabil kalınca geç.
            if d_v < (FINAL_ALIGN_DIST - 0.03):
                self.desc_stable_cnt += 1
            else:
                self.desc_stable_cnt = 0
            if self.desc_stable_cnt >= 3:
                self.state = "FINAL_ALIGN"
                self.desc_stable_cnt = 0
                self.get_logger().info(">>> DESCENDING -> FINAL_ALIGN")
            # Kör iniş tamamlanma fallback'i: görüş BLIND_DESCENT_TIMEOUT (8 s) boyunca
            # hiç dönmediyse d_v donmuş kalır ve histerezis asla tetiklenmez.
            # Bu durumda SEARCHING'e düşmek yerine fiziksel ilerlemenin gerçekleştiğini
            # varsayıp FINAL_ALIGN'a ilerle (dock'u tamamlama yönünde).
            elif (not visible) and (time.time() - self.last_seen_t) > BLIND_DESCENT_TIMEOUT:
                self.state = "FINAL_ALIGN"
                self.desc_stable_cnt = 0
                self.get_logger().warn(">>> DESCENDING -> FINAL_ALIGN (blind timeout fallback)")
        
        elif self.state == "FINAL_ALIGN":
            # Final fazda daha hafif iniş, dock noktasında -0.02'ye kadar yumuşak düşer.
            # DOCKED (vz=0) geçişindeki throttle sıçramasını minimize eder.
            vz = max(-0.08, -0.02 - 0.2 * d_v)
            # Aynı histerezis DOCKED geçişi için de: gürültü yüzünden prematüre DOCKED olmasın.
            if d_v < (DIST_DOCKED - 0.02):
                self.final_stable_cnt += 1
            else:
                self.final_stable_cnt = 0
            if self.final_stable_cnt >= 3:
                self.state = "DOCKED"
                self.final_stable_cnt = 0
                self.get_logger().info("✅ DOCKED: Görev Tamamlandı.")
            # Aynı kör iniş fallback'i: uzun süreli görüş kaybında görevi tamamla.
            # Bu aşamada vehicle dock'a çok yakın olduğu için timeout sonunda DOCKED'a geç.
            elif (not visible) and (time.time() - self.last_seen_t) > BLIND_DESCENT_TIMEOUT:
                self.state = "DOCKED"
                self.final_stable_cnt = 0
                self.get_logger().warn("✅ DOCKED (blind timeout fallback)")
        
        elif self.state == "DOCKED":
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        # ── Komut Gönderimi ve Sönümleme (Damping) ──
        if self.state != "DOCKED":
            if visible:
                if self.state in ["DESCENDING", "FINAL_ALIGN"]:
                    # İniş sırasında yatay hareketi kes (Atalet riskine karşı)
                    vx, vy = 0.0, 0.0
                else:
                    vx, vy = self.last_vx, self.last_vy
                vyaw = self.last_vyaw
            else:
                # Görüş kaybında yumuşak yavaşlama
                if self.state in ["DESCENDING", "FINAL_ALIGN"]:
                    # İniş aşamasında yatay flicker'ı önle: görünürlük/kayıp arası 0↔last_v*0.8 salınımı
                    # motorlarda gözle görülür sallanmaya sebep oluyordu. Kör iniş yalnızca dikey.
                    vx, vy, vyaw = 0.0, 0.0, 0.0
                elif self.state in ["APPROACHING", "ALIGNING"]:
                    vx, vy, vyaw = self.last_vx * 0.8, self.last_vy * 0.8, self.last_vyaw * 0.8

            # vz 1. derece low-pass (alpha=0.2 @ 25 Hz ≈ 160 ms zaman sabiti):
            # DESCENDING -> FINAL_ALIGN -> DOCKED geçişlerindeki throttle step'ini yutar,
            # buoyancy kaynaklı ani yukarı sıçramayı engeller.
            vz = 0.8 * self.last_vz_cmd + 0.2 * vz
            self.last_vz_cmd = vz

            self.send(vx, vy, vz, vyaw)
        else:
            # DOCKED: aktif olarak sıfır komut gönder. Pixhawk'ın failsafe timeout'unu
            # beklemek yerine motorları açıkça durdurur; buoyancy nedeniyle post-dock
            # yukarı sürüklenmeyi engeller. vz LPF'i de sıfıra doğru süzülür ki
            # son komutla bu tick arasında kalan küçük negatif değer de kademeli kessin.
            self.last_vz_cmd *= 0.8
            self.send(0.0, 0.0, self.last_vz_cmd, 0.0)

        # Terminal Logu (25 Hz döngüyü kirletmemek için 10 döngüde bir)
        if int(time.time() * 25) % 10 == 0:
            self.get_logger().info(
                f"[{self.state}] Vis:{visible} | MKR:{n_valid} | "
                f"x:{x_w:.2f} y:{y_w:.2f} dv:{d_v:.2f} | "
                f"vx:{vx:.2f} vy:{vy:.2f} vz:{vz:.2f}"
            )

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
