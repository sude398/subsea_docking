import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np

from . import config
from .perception import MarkerDetector, MarkerCluster
from .rov_control import PID, SmartSearch, RovInterface


# ─────────────────────────────────────────────
# FSM STATES
# ─────────────────────────────────────────────
STATE_SEARCHING   = "SEARCHING"    # marker görünmüyor
STATE_ALIGNING    = "ALIGNING"     # marker görünüyor, yaw hizalanıyor
STATE_APPROACHING = "APPROACHING"  # yaw hizalandı, yatay yaklaşma
STATE_DESCENDING  = "DESCENDING"   # platform altında, dikey iniş
STATE_DOCKED      = "DOCKED"       # kenetlendi
STATE_FAILSAFE    = "FAILSAFE"     # 5 sn+ hedef kaybı


class DockingNode(Node):

    def __init__(self):
        super().__init__('docking_node')

        self.bridge = CvBridge()

        # ── Algılama ───────────────────────────
        self.detector = MarkerDetector()
        self.cluster  = MarkerCluster()

        self.rov    = RovInterface()
        self.search = SmartSearch()

        # ── PID Kontrolcüler ───────────────────
        # pid_x  → ileri/geri hareketi kontrol eder (ey hatası)
        # pid_y  → sağ/sol hareketi kontrol eder   (ex hatası)
        # pid_yaw → yaw dönüşünü kontrol eder
        
        #   Araç yavaş yaklaşıyorsa kp artır.
        #   Araç sallanıyorsa kd artır veya rate_limit düşür.
        #   Sabit küçük sapma varsa ki artır.
        self.pid_x   = PID(0.8, 0.05, 0.20, limit=config.MAX_LATERAL, rate_limit=0.04)
        self.pid_y   = PID(0.8, 0.05, 0.20, limit=config.MAX_LATERAL, rate_limit=0.04)
        self.pid_yaw = PID(0.6, 0.02, 0.15, limit=config.MAX_YAW,     rate_limit=0.03)

        # ── State Değişkenleri ─────────────────
        self.state       = STATE_SEARCHING
        self.frame       = None
        self.last_seen_t = time.time()
        self.lock_streak = 0

        # Son bilinen değerler — blind zone'da kullanılır
        self.last_cx   = config.IMAGE_WIDTH  // 2
        self.last_cy   = config.IMAGE_HEIGHT // 2
        self.last_dist_h = 3.0
        self.last_dist_v = 1.0
        self.last_yaw    = 0.0

        # ── ROS Publisher / Subscriber ─────────
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

        self.pub_img   = self.create_publisher(Image,          '/rov/image_processed', 10)
        self.pub_state = self.create_publisher(String,         '/rov/state',           10)
        self.pub_dist  = self.create_publisher(Float32,        '/rov/distance',        10)
        # Gönderilen hız komutlarını izlemek için
        self.pub_cmd   = self.create_publisher(Float32MultiArray, '/rov/cmd_vel_debug', 10)

        # 30 Hz döngü
        self.create_timer(0.033, self.loop) #kod saniyede 30 kez çalışıyor

        self.get_logger().info("Docking Node HAZIR")

    # ─────────────────────────────────────────
    # IMAGE CALLBACK
    # ─────────────────────────────────────────
    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ─────────────────────────────────────────
    # ANA DÖNGÜ
    # ─────────────────────────────────────────
    def loop(self):
        if self.frame is None:
            return

        # Kamera ters bağlı → 180° döndür
        frame = cv2.rotate(self.frame.copy(), cv2.ROTATE_180)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detect(gray)

        is_lock, cx, cy, dist_h, dist_v, yaw, mcnt = self.cluster.compute(corners, ids)

        # Geçerli tespitlerde son bilinen değerleri güncelle
        if is_lock:
            self.last_cx     = cx
            self.last_cy     = cy
            self.last_dist_h = dist_h
            self.last_dist_v = dist_v
            self.last_yaw    = yaw

        # ── Kilit Takibi ────────────────────────
        if ids is not None and len(ids) > 0:
            self.lock_streak += 1
            self.last_seen_t  = time.time()
        else:
            self.lock_streak  = 0

        # Geçici titreme / tek frame kaybını filtrele
        confirmed_lock = self.lock_streak >= 2
        lost_time      = time.time() - self.last_seen_t

        # ── FSM ─────────────────────────────────
        self._fsm(confirmed_lock, dist_h, dist_v, lost_time)

        # ── Kontrol ─────────────────────────────
        self._control(confirmed_lock, cx, cy, dist_h, dist_v, yaw)

        # ── HUD ─────────────────────────────────
        self._hud(frame, cx, cy, dist_h, dist_v, yaw, mcnt)

        # ── Yayın ───────────────────────────────
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self.pub_state.publish(String(data=self.state))
        self.pub_dist.publish(Float32(data=float(dist_h)))

    # ─────────────────────────────────────────
    # FINITE STATE MACHINE
    # ─────────────────────────────────────────
    def _fsm(self, lock, dist_h, dist_v, lost):
        prev = self.state

        # ── Failsafe: uzun süre hedef yok ──────
        if lost > config.FAILSAFE_LOST and self.state != STATE_DOCKED:
            self.state = STATE_FAILSAFE

        elif self.state == STATE_FAILSAFE:
            if lock:
                self.state = STATE_ALIGNING
                self.search.reset()

        # ── Arama ───────────────────────────────
        elif self.state == STATE_SEARCHING:
            if lock:
                self.state = STATE_ALIGNING
                self.search.reset()

        # ── Yaw Hizalama ────────────────────────
        elif self.state == STATE_ALIGNING:
            if not lock:
                self.state = STATE_SEARCHING
            elif dist_h < config.DIST_ALIGN:
                self.state = STATE_APPROACHING

        # ── Yatay Yaklaşma ──────────────────────
        elif self.state == STATE_APPROACHING:
            if not lock:
                self.state = STATE_SEARCHING
            elif dist_h < config.DIST_DESCENDING:
                self.state = STATE_DESCENDING

        # ── Dikey İniş ──────────────────────────
        elif self.state == STATE_DESCENDING:
            if dist_v < config.DIST_DOCKED:
                # Kenetlendi
                self.state = STATE_DOCKED
                self.rov.stop()
                self.get_logger().info("✓ DOCKED — kenetleme tamamlandı")

            elif not lock and dist_h > config.DIST_BLIND_ZONE:
                # Uzaktayken lock kaybı → gerçekten kayboldu
                self.state = STATE_SEARCHING

            # dist_h <= DIST_BLIND_ZONE ve lock yok → kamera platformu göremez,
            # son bilinen hatayla devam et (aşağıda _control bunu işler)

        if prev != self.state:
            self.get_logger().info(f"FSM: {prev} → {self.state}")

    # ─────────────────────────────────────────
    # KONTROL
    # ─────────────────────────────────────────
    def _control(self, lock, cx, cy, dist_h, dist_v, yaw):

        if self.state == STATE_DOCKED:
            return

        # ── Arama / Failsafe ────────────────────
        if self.state in [STATE_SEARCHING, STATE_FAILSAFE]:
            vz, vyaw = self.search.get_velocity()
            self._send(0.0, 0.0, vz, vyaw)
            return

        if not lock and self.state != STATE_DESCENDING:
            self.rov.stop()
            return

        # ── Piksel Hata Hesabı ──────────────────
        half_w = config.IMAGE_WIDTH  / 2.0
        half_h = config.IMAGE_HEIGHT / 2.0
        fx     = config.FOCAL_LENGTH_PX

        # Kamera 50cm önde olduğu için, araç merkezi platforma hizalandığında
        # marker görüntüde merkezin ÜSTÜNDE görünmeli (kamera ileriye bakıyor).
        # Bu nedenle target_y = half_h - offset_px (işaret eksi).
        
        #   Kamera ileriye (ve 30° aşağı) bakıyor.
        #   Aracın merkezi kameradan 50cm arkada.
        #   Bu 50cm'yi görüntüde doğru yere yansıtmak için:
        #   offset_px = fx * CAMERA_FORWARD_OFFSET / dist_h
        #
        # Yakınlaştıkça offset artar (açı büyür) — bu doğru davranış.
        offset_px = fx * config.CAMERA_FORWARD_OFFSET_M / max(dist_h, 0.3)

        target_x = half_w
        target_y = half_h - offset_px   # ← EKSİ: marker görüntünün üst yarısında hedefleniyor

        # Normalize hata [-1, +1]
        ex = (cx - target_x) / half_w
        ey = (cy - target_y) / half_h

        # Deadzone — küçük sapmaları sıfırla, titreme önle
        if abs(cx - target_x) < config.DEADZONE_PX:
            ex = 0.0
        if abs(cy - target_y) < config.DEADZONE_PX:
            ey = 0.0

        # Blind zone'da lock yoksa son bilinen hatayı kullan
        if not lock:
            last_ex = (self.last_cx - target_x) / half_w
            last_ey = (self.last_cy - target_y) / half_h
            ex, ey  = last_ex, last_ey
            yaw     = self.last_yaw

        # ── ALIGNING: sadece yaw düzelt ─────────
        if self.state == STATE_ALIGNING:
            vyaw = self.pid_yaw.update(-yaw)
            self._send(0.0, 0.0, 0.0, vyaw)

        # ── APPROACHING: yatay yaklaş ───────────
        elif self.state == STATE_APPROACHING:
            vx   = self.pid_x.update(ey)    # ey → ileri/geri
            vy   = self.pid_y.update(ex)    # ex → sağ/sol
            vyaw = self.pid_yaw.update(-yaw)
            self._send(vx, vy, 0.0, vyaw)

        # ── DESCENDING: aşağı in ────────────────
        elif self.state == STATE_DESCENDING:
            vx   = self.pid_x.update(ey)
            vy   = self.pid_y.update(ex)

            # Yakın mesafede yaw düzeltmesi thruster'ları yatay iter
            # ve kenetlemeyi zorlaştırabilir. 5° altında yaw kontrolünü kes.
            if abs(yaw) > np.deg2rad(5):
                vyaw = self.pid_yaw.update(-yaw)
            else:
                vyaw = 0.0

            # dist_v kullan: dikey mesafe azaldıkça hız düşür (%30 → %100 arası)
            ratio = np.clip(
                (dist_v - config.DIST_DOCKED) /
                max(self.last_dist_v - config.DIST_DOCKED, 0.01),
                0.0, 1.0
            )
            vz = -config.MAX_VERTICAL * (0.3 + 0.7 * ratio)  # negatif = aşağı

            self._send(vx, vy, vz, vyaw)

    # ─────────────────────────────────────────
    # HIZ GÖNDER + DEBUG YAYINI
    # ─────────────────────────────────────────
    def _send(self, vx, vy, vz, vyaw):
        self.rov.send_velocity(vx, vy, vz, vyaw)
        msg      = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(vz), float(vyaw)]
        self.pub_cmd.publish(msg)

    # ─────────────────────────────────────────
    # HUD
    # ─────────────────────────────────────────
    def _hud(self, img, cx, cy, dist_h, dist_v, yaw, mcnt):
        half_w    = config.IMAGE_WIDTH  // 2
        half_h    = config.IMAGE_HEIGHT // 2
        fx        = config.FOCAL_LENGTH_PX
        offset_px = int(fx * config.CAMERA_FORWARD_OFFSET_M / max(dist_h, 0.3))

        tx = half_w
        ty = half_h - offset_px   # kontrol ile aynı hedef

        # Kırmızı ✛ → platform marker merkezi (algılanan)
        # Mavi  ✕ → araç merkezinin platforma hizalanması gereken nokta
        # Sarı  ─ → hata vektörü
        cv2.drawMarker(img, (cx, cy), (0, 0, 255),   cv2.MARKER_CROSS,         40, 2)
        cv2.drawMarker(img, (tx, ty), (255, 0, 0),   cv2.MARKER_TILTED_CROSS,  25, 2)
        cv2.line(img, (cx, cy), (tx, ty), (0, 255, 255), 2)

        yaw_deg = np.rad2deg(yaw)

        cv2.putText(img, f"State:  {self.state}",          (20,  35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.putText(img, f"Dist H: {dist_h:.2f} m",        (20,  70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.putText(img, f"Dist V: {dist_v:.2f} m",        (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.putText(img, f"Yaw:    {yaw_deg:.1f} deg",     (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.putText(img, f"Markers:{mcnt}",                (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

    # ─────────────────────────────────────────
    # SHUTDOWN
    # ─────────────────────────────────────────
    def destroy_node(self):
        self.rov.stop()
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


if __name__ == '__main__':
    main()
