import os
import time
import math
import sys
import threading

# Add current folder to path to make sure local imports work perfectly
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node

os.environ['MAVLINK20'] = '1'

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pymavlink import mavutil

try:
    from updated_config import (
        IMAGE_WIDTH, IMAGE_HEIGHT,
        MAX_LATERAL, MAX_YAW,
        DIST_ALIGN, DIST_DESCENDING, FINAL_ALIGN_DIST, DIST_DOCKED,
        PRECISION_THRESH, HOVER_TIME, FAILSAFE_LOST, CONFIDENCE_THRESHOLD,
        CAMERA_OFFSET_Y, SERIAL_PORT, BAUD_RATE,
        CAMERA_DEVICE, CAMERA_FLIP, GSTREAMER_PORT, GSTREAMER_TARGET_IP
    )
    from updated_controller import PID, Kalman2D
    from updated_vision import Vision
    from camera_streamer import CameraStreamer
except ImportError:
    from config import (
        IMAGE_WIDTH, IMAGE_HEIGHT,
        MAX_LATERAL, MAX_YAW,
        DIST_ALIGN, DIST_DESCENDING, FINAL_ALIGN_DIST, DIST_DOCKED,
        PRECISION_THRESH, HOVER_TIME, FAILSAFE_LOST, CONFIDENCE_THRESHOLD,
        CAMERA_OFFSET_Y, SERIAL_PORT, BAUD_RATE
    )
    from controller import PID, Kalman2D
    from vision import Vision
    # Fallbacks if running outside our integrated bundle
    CAMERA_DEVICE = '/dev/video2'
    CAMERA_FLIP = False
    GSTREAMER_PORT = 5603
    GSTREAMER_TARGET_IP = '10.42.0.1'

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')
        
        # ROS2 Parameters (CAMERA_DEVICE is used as the default path)
        self.declare_parameter('video_source', CAMERA_DEVICE)
        self.video_source = self.get_parameter('video_source').value

        # [İYİLEŞTİRME] CameraStreamer Integration (Threaded Capture & H.264 UDP Stream to Surface)
        self.get_logger().info(f"Connecting to Camera: {self.video_source} ...")
        self.cam = CameraStreamer(
            self.video_source, 
            GSTREAMER_PORT, 
            'BOTTOM', 
            flip=CAMERA_FLIP, 
            monster_ip=GSTREAMER_TARGET_IP
        )

        # Wait for camera stream to become active
        timeout = time.time() + 10.0
        while time.time() < timeout:
            if self.cam.is_ready():
                break
            time.sleep(0.1)

        if not self.cam.is_ready():
            self.get_logger().error("❌ Camera capture thread failed to start!")
        else:
            self.get_logger().info("✅ Camera capture & GStreamer streamer started.")

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, '/docking/camera_view', 10)

        self.vision = Vision()
        self.kalman = Kalman2D()

        # Sallanmayı önlemek için güncellenmiş PID katsayıları
        self.pid_x = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_y = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_yaw = PID(0.5, 0.02, 0.3, MAX_YAW)

        self.state = "SEARCHING"
        self.blind_start_t = None
        self.confirmed_4_markers = False
        self.approach_confidence = 0
        self.search_start_t = None
        self.approach_start_t = None
        self.local_search_start_t = None
        self.descending_start_t = None
        self.final_align_start_t = None

        self.last_seen_t = time.time()
        self.last_loop_t = time.time()
        self.d_v = 999.0
        self.last_vz_cmd = 0.0
        self.armed = False
        self._running = True

        # Yumuşak Kilit için son komutları tutan değişkenler
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0

        # [Risk 2 Düzeltmesi] Son bilinen yatay mesafe (görüş koptuğunda 999.0 yerine gerçekçi değer kullanılır)
        self.last_d_h = 999.0

        # MAVLink Threading / Command values
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw = 0.0

        # Pixhawk Serial connection
        self.master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        self.master.wait_heartbeat(timeout=5)
        self.get_logger().info("✅ Pixhawk Bağlantısı OK")

        # [İYİLEŞTİRME] Failsafe settings to prevent manual control loss or overrides
        # (pipeline_dual_camera.py example connect function pattern)
        for name, val in [("FS_PILOT_EN", 0), ("FS_GCS_EN", 0),
                          ("ARMING_CHECK", 0), ("BRD_SAFETYENABLE", 0)]:
            try:
                self._set_param(name, val)
            except Exception as e:
                self.get_logger().warn(f"Failed to set param {name}: {e}")

        self._set_mode("ALT_HOLD")
        self._arm()

        # [İYİLEŞTİRME] Background Heartbeat and Command Thread
        # Sends heartbeats & manual control coordinates at 20 Hz (50ms interval)
        threading.Thread(target=self._heartbeat_thread, daemon=True).start()

        # ROS2 main loop timer (25 Hz)
        self.create_timer(0.04, self.loop)

    def _set_param(self, name, value):
        if self.master:
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                name.encode('utf-8'), value,
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
                self.get_logger().info(f"✅ Uçuş Modu {mode} olarak ayarlandı.")
            except Exception as e:
                self.get_logger().error(f"Mod ayarlanırken hata: {e}")
 
    def _arm(self):
        """Aracı arm eder ve arm durumunu doğrular."""
        try:
            self.master.arducopter_arm()
            # Arm onayını bekle (maks 5 saniye)
            ack = self.master.motors_armed_wait(timeout=5)
            if ack:
                self.armed = True
                self.get_logger().info("✅ Araç ARM edildi.")
            else:
                self.armed = False
                self.get_logger().error("❌ ARM başarısız! Pixhawk onaylamadı.")
        except Exception as e:
            self.armed = False
            self.get_logger().error(f"ARM hatası: {e}")

    def _disarm(self):
        """Aracı disarm eder (Gelişmiş Pipeline Örneği Uyumu)."""
        if self.master:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                self.armed = False
                self.get_logger().info("✅ Araç DISARM edildi.")
            except Exception as e:
                self.get_logger().error(f"DISARM hatası: {e}")

    def send(self, vx, vy, vz, yaw):
        # [Risk 15] Tüm değerleri güvenli aralığa kısıtla (bug durumunda taşma koruması)
        vx  = max(-1.0, min(1.0, vx))
        vy  = max(-1.0, min(1.0, vy))
        vz  = max(-1.0, min(1.0, vz))
        yaw = max(-1.0, min(1.0, yaw))
        
        # Update command variables for background thread
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_vz = vz
        self.cmd_yaw = yaw

    def _heartbeat_thread(self):
        """Arka planda Pixhawk'a 20 Hz frekansta Heartbeat ve son hareket komutlarını gönderir."""
        while self._running and rclpy.ok():
            try:
                if self.master:
                    # Heartbeat gönder
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                    )
                    
                    # Son komutları manual_control_send üzerinden Pixhawk'a ilet
                    self.master.mav.manual_control_send(
                        self.master.target_system,
                        int(self.cmd_vx * 1000),
                        int(self.cmd_vy * 1000),
                        int(500 + self.cmd_vz * 1000),
                        int(self.cmd_yaw * 1000),
                        0
                    )
            except Exception as e:
                pass
            time.sleep(0.05)

    def loop(self):
        now = time.time()
        dt = min(now - self.last_loop_t, 0.1)
        self.last_loop_t = now
        if not self.armed:
            self.get_logger().warn("Araç ARM değil, döngü atlandı.")
            return

        # [İYİLEŞTİRME] Thread-safe capture via CameraStreamer copy frame
        frame = self.cam.get_frame()
        if frame is None:
            # Görüntü gelene kadar komut göndermeyi sürdür
            return

        frame, visible, locked, n_valid, target_tvec, target_yaw, vision_d_v = self.vision.process(frame)

        # HEDEF KAYMASI (TARGET SNAP) KORUMASI:
        # Araç 4 marker onayı aldıysa, artık hedefin köşede değil merkezde olduğunu biliyordur.
        # Bu saatten sonra kısıtlı görüş açısıyla (sadece 1 marker görerek) hedefin köşeye kaymasını engelle!
        # "locked == True", merkezin kesin olarak hesaplanabildiği (2 diyagonal, 3 veya 4 mkr) anları temsil eder.
        if self.confirmed_4_markers and visible and not locked:
            visible = False  # Köşeyi merkez zannetme, hedefi yoksay (Kör Uçuşa Geç)
            cv2.putText(frame, "IGNORING CORNER (SNAP PROTECTION)", (10, 150), 2, 0.6, (0, 0, 255), 2)

        # İNİŞ MODU GÜVENLİĞİ (Kesin Merkez Koruması):
        # İniş aşamasındayken bozulmuş/yanıltıcı marker'lardan (false positive) etkilenmemek
        # için sadece merkezin kesin hesaplanabildiği (locked=True) anlarda kameraya güven.
        # Bu sayede 1 marker veya 2 yan-yana marker görülürse reddedilir, 
        # ancak 2 diyagonal, 3 veya 4 marker görülürse kabul edilir.
        if self.state in ["DESCENDING", "FINAL_ALIGN"] and visible:
            if not locked:
                visible = False
                cv2.putText(frame, "UNLOCKED IGNORED (DESCENDING)", (10, 175), 2, 0.6, (0, 0, 255), 2)

        # Başlangıç değerleri
        # [Risk 2 Düzeltmesi] d_h, görüş koptuğunda 999.0 yerine son bilinen mesafeyle başlar.
        # Bu sayede APPROACHING modunda tek kötü kare iniş iznini engelleyemez.
        x_w, y_w, y_error, d_h = 0.0, 0.0, 0.0, self.last_d_h
        vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        if visible and target_tvec is not None:
            # [Kalman Düzeltmesi - Risk 3] Görüş uzun süre koptuktan sonra tekrar sağlandığında,
            # Kalman filtresinin eski yanlış konumu reddetmesini (outlier lock) engellemek için sıfırla.
            if time.time() - self.last_seen_t > 1.5:
                self.kalman.initialized = False
                self.get_logger().warn(">>> Görüş uzun süre koptu, Kalman sıfırlandı.")
                
            self.last_seen_t = time.time()
            self.d_v = vision_d_v
            raw_x, raw_y = self.vision.compute_world(target_tvec)
            x_w, y_w = self.kalman.update([raw_x, raw_y])
            
            # Kamera öndeyse, hedef kameranın gerisinde (-Y) kalmalıdır.
            # Y ekseni hatası (hedef - (-ofset)) -> y_w + CAMERA_OFFSET_Y
            y_error = y_w + CAMERA_OFFSET_Y
            
            d_h = np.sqrt(x_w**2 + y_error**2)
            self.last_d_h = d_h  # [Risk 2] Son geçerli yatay mesafeyi kaydet

            self.last_vx = self.pid_x.update(y_error)
            self.last_vy = self.pid_y.update(x_w)
            self.last_vyaw = self.pid_yaw.update(-target_yaw)
            
        elif self.state in ["APPROACHING", "DESCENDING", "FINAL_ALIGN"]:
            # [Kör Uçuş / Dead Reckoning]
            # Yatay konum güncellemesi
            dx_est = self.last_vy * dt
            dy_est = self.last_vx * dt
            
            self.kalman.x[0,0] -= dx_est
            self.kalman.x[1,0] -= dy_est
            
            x_w = self.kalman.x[0,0]
            y_w = self.kalman.x[1,0]
            y_error = y_w + CAMERA_OFFSET_Y
            d_h = np.sqrt(x_w**2 + y_error**2)
            self.last_d_h = d_h
            
            # Dikey konum güncellemesi (sadece iniş modlarında)
            if self.state in ["DESCENDING", "FINAL_ALIGN"]:
                expected_vz = -0.15 if self.state == "DESCENDING" else -0.08
                self.d_v += expected_vz * dt
                self.d_v = max(0.0, self.d_v)

        # ── Güven ve Onay Mekanizması ──
        if locked:
            self.approach_confidence = min(CONFIDENCE_THRESHOLD + 10, self.approach_confidence + 1)
        else:
            # İniş aşamasındaysak güven puanını düşürme (marker kaybı normaldir)
            if self.state not in ["DESCENDING", "FINAL_ALIGN"]:
                self.approach_confidence = max(0, self.approach_confidence - 1)

        # Onay Kilidi: Sadece Searching modunda sıfırlanır
        if self.approach_confidence >= CONFIDENCE_THRESHOLD and not self.confirmed_4_markers:
            self.confirmed_4_markers = True
            self.get_logger().info("✅ Hedef Merkez Kilitlendi. İniş yetkisi verildi.")

        # ── FSM (Sonlu Durum Makinesi) ──
        if self.state == "SEARCHING":
            # [Risk 8 Düzeltmesi] SEARCHING'e her girişte tüm hafızayı sıfırla.
            # Artık confirmed_4_markers koşuluna bağlı değil — onaysız durumda da temizlenir.
            if self.search_start_t is None:
                self.search_start_t = time.time()
                self.confirmed_4_markers = False
                self.approach_confidence = 0
                self.last_d_h = 999.0
                self.kalman.initialized = False
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_yaw.reset()
                self.last_vx = 0.0
                self.last_vy = 0.0
                self.last_vyaw = 0.0
                self.descending_start_t = None
                self.final_align_start_t = None
                self.get_logger().info(">>> SEARCHING: Tüm yetkiler ve hafıza sıfırlandı.")

            if visible:
                self.state = "ALIGNING"
                self.search_start_t = None
                self.get_logger().info(">>> SEARCHING -> ALIGNING")
            else:
                # [Detaylı Arama - Gelişmiş SEARCHING]
                elapsed = time.time() - self.search_start_t
                
                if elapsed < 5.0:
                    # Aşama 1: Sadece hafifçe yüksel (FOV - Görüş Açısını Büyütmek İçin)
                    vx, vy, vyaw = 0.0, 0.0, 0.0
                    vz = 0.15
                    cv2.putText(frame, "SEARCH: CLIMBING", (10, 150), 2, 0.6, (0, 165, 255), 2)
                elif elapsed < 15.0:
                    # Aşama 2: Yükselmeyi durdur ve kendi etrafında dönerek tara
                    vx, vy, vz = 0.0, 0.0, 0.0
                    vyaw = 0.2
                    cv2.putText(frame, "SEARCH: 360 SCAN", (10, 150), 2, 0.6, (0, 165, 255), 2)
                else:
                    # Aşama 3: Gelişmiş Arama (Lawnmower / Genişleyen Çokgen)
                    # 15. saniyeden sonra devreye girer.
                    adv_t = elapsed - 15.0
                    
                    # 15 saniyelik sürekli kendini tekrar eden bir döngü
                    cycle_t = adv_t % 15.0
                    
                    if cycle_t < 5.0:
                        # Döngünün ilk 5 saniyesi: Yeni yöne doğru ileri git (Genişlemek için hızı yavaşça artırıyoruz)
                        fwd_speed = min(0.35, 0.15 + (adv_t * 0.002))
                        vx = fwd_speed
                        vy, vz, vyaw = 0.0, 0.0, 0.0
                        cv2.putText(frame, f"ADV SEARCH: FORWARD ({fwd_speed:.2f})", (10, 150), 2, 0.6, (0, 0, 255), 2)
                    else:
                        # Döngünün kalan 10 saniyesi: İlerlemeyi durdur ve kendi etrafında dönerek 360 derece tara.
                        # (Dönüş bittiğinde araç sağa doğru yeni bir yöne bakıyor olacak ve tekrar ileri gidecek)
                        vx, vy, vz = 0.0, 0.0, 0.0
                        vyaw = 0.2
                        cv2.putText(frame, "ADV SEARCH: 360 SCAN", (10, 150), 2, 0.6, (0, 0, 255), 2)
        
        elif self.state == "ALIGNING":
            if not visible and (time.time() - self.last_seen_t > 3.0):
                self.state = "LOCAL_SEARCH"
                self.local_search_start_t = time.time()
                self.get_logger().warn(">>> Hedef Kayboldu: ALIGNING -> LOCAL_SEARCH")
            elif visible and d_h < DIST_ALIGN:
                # [Risk 12] Sadece görüş varken mesafe kontrolü yap.
                # last_d_h'ın eski değeriyle erken geçiş yapılmasını önler.
                self.state = "APPROACHING"
                self.approach_start_t = time.time()
                self.get_logger().info(">>> ALIGNING -> APPROACHING")
        
        elif self.state == "APPROACHING":
            if self.confirmed_4_markers and d_h < DIST_DESCENDING:
                # İNİŞ İZNİ: 4 marker önceden onaylandı, yatayda merkeze varıldı (kör bile olsa in)
                # [Risk 9] Geçiş anında d_v'yi sabitle (dead reckoning doğru başlasın)
                if self.d_v > 10.0:
                    self.d_v = 2.0  # Güvenlik: d_v çok eski/büyükse makul değere çek
                self.state = "DESCENDING"
                self.get_logger().warn(f"!!! DESCENDING STARTED (d_v={self.d_v:.2f}m)")
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
                elif self.approach_start_t and (time.time() - self.approach_start_t) > 20.0:
                    # [Timeout Düzeltmesi - Risk 1] 4 saniye çok kısaydı, yaklaşma için fiziksel olarak 
                    # yeterli olan 20 saniye tanındı. Böylece hedefi bulmuşken erken pes etmez.
                    self.state = "LOCAL_SEARCH"
                    self.local_search_start_t = time.time()
                    self.get_logger().info(">>> APPROACHING TIMEOUT -> LOCAL_SEARCH: SARMAL ARAMA BASLADI")
                    
        elif self.state == "LOCAL_SEARCH":
            if not visible and (time.time() - self.local_search_start_t > 20.0):
                self.state = "SEARCHING"
                self.search_start_t = None 
                self.get_logger().warn(">>> Hedef Kayboldu: LOCAL_SEARCH -> SEARCHING")
            elif locked:
                # Gerçek merkez (2 diyagonal, 3 veya 4 marker) bulunduysa hedefe geri kilitlen
                self.state = "APPROACHING"
                self.approach_start_t = time.time() # kronometreyi sıfırla
                self.get_logger().info(">>> LOCAL_SEARCH -> APPROACHING: Gerçek Merkez Bulundu")
            else:
                t_search = time.time() - self.local_search_start_t
                # [Risk 13] İlk 2 saniye yüksel (maks ~0.3m), sonra sadece sarmal yap.
                # Böylece birden fazla LOCAL_SEARCH döngüsünde araç kontrolsüz yükselmez.
                if t_search < 2.0:
                    vz = 0.15  # Yüksel (FOV genişlet)
                else:
                    vz = 0.0   # Yeterince yükseldin, sadece etrafı tara
                # Sarmal hız (dairesel hareket)
                vx = 0.2 * math.cos(t_search)
                vy = 0.2 * math.sin(t_search)
                cv2.putText(frame, "LOCAL SEARCH (SPIRAL)...", (10, 200), 2, 0.6, (0, 165, 255), 2)
        
        elif self.state == "DESCENDING":
            vz = -0.15
            # DESCENDING'e ilk girişte zamanı kaydet
            if self.descending_start_t is None:
                self.descending_start_t = time.time()

            # Timeout: 15 saniyede FINAL_ALIGN'a zorla (d_v kameradan gelmiyorsa güvenlik)
            if time.time() - self.descending_start_t > 15.0:
                self.state = "FINAL_ALIGN"
                self.get_logger().warn(">>> DESCENDING TIMEOUT -> FINAL_ALIGN")
            elif self.d_v < FINAL_ALIGN_DIST:
                self.state = "FINAL_ALIGN"
                self.get_logger().info(">>> DESCENDING -> FINAL_ALIGN")
        
        elif self.state == "FINAL_ALIGN":
            vz = -0.08
            if self.final_align_start_t is None:
                self.final_align_start_t = time.time()

            # Timeout: 10 saniyede DOCKED durumuna zorla (güvenlik durdurması)
            if time.time() - self.final_align_start_t > 10.0:
                self.state = "DOCKED"
                self.get_logger().warn(">>> FINAL_ALIGN TIMEOUT -> DOCKED")
            elif self.d_v < DIST_DOCKED:
                self.state = "DOCKED"
                self.get_logger().info("✅ DOCKED: Görev Tamamlandı.")
        
        elif self.state == "DOCKED":
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
            # [İYİLEŞTİRME] Explicit disarm on success transition
            if self.armed:
                self.send(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.5)
                self._disarm()

        # ── Komut Gönderimi ve Sönümleme (Damping) ──
        if self.state == "LOCAL_SEARCH":
            # FSM bloğunda hesaplanan sarmal vx, vy, vz komutları geçerlidir
            # Yönelimi (Yaw) sabit tut
            vyaw = self.last_vyaw
        elif self.state == "SEARCHING":
            # FSM bloğunda atanan arama hızları geçerlidir
            pass
        elif self.state == "DOCKED":
            # [Risk 4 Düzeltmesi] DOCKED durumunda her döngüde sıfır komutu gönder.
            # Pixhawk'ın failsafe'e düşmesini veya son komutu tutmasını engeller.
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
        else:
            # ALIGNING, APPROACHING, DESCENDING, FINAL_ALIGN modları
            if visible:
                # Hedef görülüyorsa PID komutlarını kullan
                vx, vy = self.last_vx, self.last_vy
                vyaw = self.last_vyaw
            elif self.state == "APPROACHING":
                # APPROACHING'de görüş koptuğunda son komutu koru (Hold Last Command).
                # Araç son bildiği yöne doğru ilerlemeye devam eder, tamamen durmaz.
                # Timeout veya iniş koşulu gelene kadar hedefe doğru hareket sürer.
                vx, vy, vyaw = self.last_vx, self.last_vy, self.last_vyaw
            else:
                # ALIGNING, DESCENDING, FINAL_ALIGN: Sönümleme (Damping) uygula.
                # Her döngüde hızı %20 azaltarak güvenli şekilde sıfıra yaklaştır.
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
        
        # [İYİLEŞTİRME] Send processed frame to surface via GStreamer UDP Stream
        self.cam.send_debug(frame)

        # Görüntüyü ROS2 Topic Üzerinden Yayınla (Alternatif/Debug olarak korundu)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_img.publish(img_msg)
        except: 
            pass

    def destroy_node(self):
        """[İYİLEŞTİRME] ROS2 Node kapandığında güvenli kapatma, disarm ve stream durdurma"""
        self._running = False
        self.get_logger().info("Durduruluyor, motorlar disarm ediliyor...")
        self.send(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.2)
        if self.armed:
            self._disarm()
        if hasattr(self, 'cam'):
            self.cam.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
