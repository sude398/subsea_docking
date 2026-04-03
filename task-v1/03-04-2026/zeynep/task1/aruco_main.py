import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np

from . import config
from .perception import MarkerDetector, MarkerCluster
from .rov_control import PID, SmartSearch, RovInterface


# ─────────────────────────────────────────────
# FSM STATES
# ─────────────────────────────────────────────
STATE_SEARCHING   = "SEARCHING" # no markers are seen
STATE_ALIGNING    = "ALIGNING" # at least one marker is seen, but not aligned
STATE_APPROACHING = "APPROACHING" # aligned but not close enough
STATE_DESCENDING  = "DESCENDING" # close enough to descend, but not docked yet
STATE_DOCKED      = "DOCKED" # docked (distance < 0.2m)
STATE_FAILSAFE    = "FAILSAFE" # lost target for more than 3 seconds


class DockingNode(Node):

    def __init__(self):
        super().__init__('docking_node')

        self.bridge = CvBridge()

        # ── Vision ─────────────────────────────
        self.detector = MarkerDetector()
        self.cluster  = MarkerCluster()

        # ── ROV ────────────────────────────────
        self.rov = RovInterface()
        self.search = SmartSearch()

        # ── PID controllers ────────────────────
        # araç çok yavaş yaklaşıyorsa p yi arttır
        # araç sürekli sallanıyorsa d yi arttır
        # araç hedefte duruyor ama 5-10cm sapma varsa i yi arttır
        
        self.pid_x   = PID(0.8, 0.05, 0.20, limit=config.MAX_LATERAL, rate_limit=0.04)
        self.pid_y   = PID(0.8, 0.05, 0.20, limit=config.MAX_LATERAL, rate_limit=0.04)
        self.pid_yaw = PID(0.6, 0.02, 0.15, limit=config.MAX_YAW, rate_limit=0.03)

        # ── State ──────────────────────────────
        self.state = STATE_SEARCHING
        self.frame = None

        self.current_dist = 5.0
        self.last_seen_t = time.time()
        self.lock_streak = 0

        # ── ROS ────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

        self.pub_img   = self.create_publisher(Image, '/rov/image_processed', 10)
        self.pub_state = self.create_publisher(String, '/rov/state', 10)
        self.pub_dist  = self.create_publisher(Float32, '/rov/distance', 10)

        self.create_timer(0.033, self.loop) # kod saniyede 30 kez çalışır

        self.get_logger().info("Docking Node READY")

    # IMAGE CALLBACK
    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # MAIN LOOP
    def loop(self):

        if self.frame is None:
            return

        frame_raw = cv2.rotate(self.frame.copy(), cv2.ROTATE_180)

        # ── IPM ─────────────────────────────
        ipm_view = cv2.warpPerspective(
            frame_raw,
            config.IPM_MATRIX,
            (config.IMAGE_WIDTH, config.IMAGE_HEIGHT),
        )

        # ── Detection ───────────────────────
        gray_ipm = cv2.cvtColor(ipm_view, cv2.COLOR_BGR2GRAY)
        gray_raw = cv2.cvtColor(frame_raw, cv2.COLOR_BGR2GRAY)

        corners_ipm, ids_ipm, _ = self.detector.detect(gray_ipm)
        corners_raw, ids_raw, _ = self.detector.detect(gray_raw)

        # ── Cluster ──────────────────────────
        is_lock, cx, cy, dist, yaw, mcnt = self.cluster.compute(
            corners_ipm, ids_ipm, corners_raw, self.current_dist
        )

        self.current_dist = dist

        # ── Lock tracking ───────────────────
        if ids_ipm is not None and len(ids_ipm) > 0:
            self.lock_streak += 1
            self.last_seen_t = time.time()
        else:
            self.lock_streak = 0

        confirmed_lock = self.lock_streak >= 2
        lost_time = time.time() - self.last_seen_t

        # ── FSM ─────────────────────────────
        self._fsm(confirmed_lock, dist, lost_time)

        # ── CONTROL ─────────────────────────
        self._control(confirmed_lock, cx, cy, dist, yaw)

        # ── HUD ─────────────────────────────
        self._hud(ipm_view, cx, cy, dist, yaw)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(ipm_view, 'bgr8'))

        self.pub_state.publish(String(data=self.state))
        self.pub_dist.publish(Float32(data=float(dist)))


    # FINITE STATE MACHINE
    def _fsm(self, lock, dist, lost):

        prev = self.state

        if lost > config.FAILSAFE_LOST:
            self.state = STATE_FAILSAFE

        elif self.state == STATE_FAILSAFE:
            if lock:
                self.state = STATE_ALIGNING
                self.search.reset()

        elif self.state == STATE_SEARCHING:
            if lock:
                self.state = STATE_ALIGNING
                self.search.reset()

        elif self.state == STATE_ALIGNING:
            if not lock:
                self.state = STATE_SEARCHING
            elif dist < config.DIST_ALIGN:
                self.state = STATE_APPROACHING

        elif self.state == STATE_APPROACHING:
            if not lock:
                self.state = STATE_SEARCHING
            elif dist < config.DIST_DESCENDING:
                self.state = STATE_DESCENDING

        elif self.state == STATE_DESCENDING:
            if not lock:
                self.state = STATE_SEARCHING
            elif dist < config.DIST_DOCKED:
                self.state = STATE_DOCKED
                self.rov.stop()
                self.get_logger().info("DOCKED")

        if prev != self.state:
            self.get_logger().info(f"{prev} → {self.state}")

    # ─────────────────────────────
    # CONTROL
    # ─────────────────────────────
    def _control(self, lock, cx, cy, dist, yaw):

        if self.state == STATE_DOCKED:
            return

        if self.state in [STATE_SEARCHING, STATE_FAILSAFE]:
            vz, vyaw = self.search.get_velocity()
            self.rov.send_velocity(0, 0, vz, vyaw)
            return

        if not lock:
            self.rov.stop()
            return

        half_w = config.IMAGE_WIDTH / 2
        half_h = config.IMAGE_HEIGHT / 2

        # aracın istasyonu kameraya göre değil gövdesine göre ortalamasını sağlar
        CAMERA_OFFSET = 0.50 # Kameranın merkeze olan gerçek mesafesini (m) güncelle
        fx = config.FOCAL_LENGTH_PX

        offset_px = (fx * CAMERA_OFFSET) / max(dist, 0.3)

        target_x = half_w
        target_y = half_h + offset_px

        ex = (cx - target_x) / half_w
        ey = (cy - target_y) / half_h

        if abs(cx - target_x) < config.DEADZONE_PX:
            ex = 0
        if abs(cy - target_y) < config.DEADZONE_PX:
            ey = 0

        # ── ALIGN ─────────────────────────
        if self.state == STATE_ALIGNING:
            vyaw = self.pid_yaw.update(-yaw)
            self.rov.send_velocity(0, 0, 0, vyaw)

        # ── APPROACH ──────────────────────
        elif self.state == STATE_APPROACHING:
            vx = self.pid_y.update(ey)
            vy = self.pid_x.update(ex)
            vyaw = self.pid_yaw.update(-yaw)

            self.rov.send_velocity(vx, vy, 0, vyaw)

        # ── DESCEND ───────────────────────
        elif self.state == STATE_DESCENDING:
            vx = self.pid_y.update(ey)
            vy = self.pid_x.update(ex)
            vyaw = self.pid_yaw.update(-yaw) # İniş anında burnu sabit tut

            ratio = np.clip(
                (dist - config.DIST_DOCKED) /
                (config.DIST_DESCENDING - config.DIST_DOCKED),
                0, 1
            )

            # Robot yaklaştıkça vz hızı %30 ile %100 arasında değişir
            vz = -config.MAX_VERTICAL * (0.3 + 0.7 * ratio)

            self.rov.send_velocity(vx, vy, vz, vyaw)

    # HUD
    def _hud(self, img, cx, cy, dist, yaw):

        half_w = config.IMAGE_WIDTH // 2
        half_h = config.IMAGE_HEIGHT // 2

        CAMERA_OFFSET = 0.20
        fx = config.FOCAL_LENGTH_PX

        offset_px = int((fx * CAMERA_OFFSET) / max(dist, 0.3))

        tx = half_w
        ty = half_h + offset_px

        # kırmızı artı -> platformun merkezi
        # mavi x -> hedefin merkezi
        # sarı çizgi -> hedefe doğru yön

        cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.drawMarker(img, (tx, ty), (255, 0, 0), cv2.MARKER_TILTED_CROSS, 25, 2)
        cv2.line(img, (cx, cy), (tx, ty), (0, 255, 255), 2)

        cv2.putText(img, f"State: {self.state}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.putText(img, f"Dist: {dist:.2f}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    # SHUTDOWN SAFE
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