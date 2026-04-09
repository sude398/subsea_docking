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


# ─────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────
class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # ── Kamera Parametresi Tanımlama ──
        self.declare_parameter('video_source', 0)
        self.video_source = self.get_parameter('video_source').value

        # Jetson USB Kamera (Eğer CSI ise GStreamer pipeline yazmanız gerekebilir)
        self.cap = cv2.VideoCapture(self.video_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ROS 2 Image Publisher (Jetson -> PC Görüntü Aktarımı)
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, '/docking/camera_view', 10)

        self.vision = Vision()
        self.kalman = Kalman2D()

        self.pid_x = PID(1.2, 0.1, 0.25, MAX_LATERAL)
        self.pid_y = PID(1.2, 0.1, 0.25, MAX_LATERAL)
        self.pid_yaw = PID(0.8, 0.05, 0.2, MAX_YAW)

        self.state = "SEARCHING"
        self.hover_start = None
        self.is_settled = False

        self.last_seen_t = time.time()
        self.armed = False

        self.master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        self.master.wait_heartbeat(timeout=5)
        self.get_logger().info("✅ Pixhawk Bağlantısı OK")

        # Otonom hareket için Pixhawk'ı MANUAL moda al
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

        # ── Vision ──
        frame, visible, locked, n_valid, target_tvec, target_yaw, d_v = self.vision.process(frame)

        x_w, y_w = 0.0, 0.0
        d_h = 0.0

        if visible and target_tvec is not None:
            self.last_seen_t = time.time()
            raw_x, raw_y = self.vision.compute_world(target_tvec)
            x_w, y_w = self.kalman.update([raw_x, raw_y])
            d_h = np.sqrt(x_w**2 + y_w**2)

        # ── ARM Kontrolü ──
        if self.master and not self.armed:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component, 400, 0, 1, 21196, 0,0,0,0,0)
            if self.master.motors_armed():
                self.armed = True
                self.get_logger().info("⚔️ Araç ARMED!")

        # ── Failsafe ──
        if time.time() - self.last_seen_t > FAILSAFE_LOST:
            visible = False
            self.state = "SEARCHING"

        # ── FSM ──
        if self.state == "SEARCHING" and visible:
            self.state = "ALIGNING"
        elif self.state == "ALIGNING" and d_h < DIST_ALIGN:
            self.state = "APPROACHING"
        elif self.state == "APPROACHING" and d_h < DIST_DESCENDING:
            self.state = "DESCENDING"
        elif self.state == "DESCENDING" and d_h < FINAL_ALIGN_DIST:
            self.state = "FINAL_ALIGN"
            self.hover_start = time.time()
            self.is_settled = False
        elif self.state == "FINAL_ALIGN" and d_v < DIST_DOCKED:
            self.state = "DOCKED"

        # ── Kontrol ──
        if self.state == "DOCKED":
            self.send(0,0,0,0)
        else:
            vx = self.pid_x.update(y_w)
            vy = self.pid_y.update(x_w)
            vyaw = self.pid_yaw.update(-target_yaw)
            vz = 0.0

            if self.state == "DESCENDING":
                vz = -0.1
            elif self.state == "FINAL_ALIGN":
                if not self.is_settled:
                    if time.time() - self.hover_start < HOVER_TIME:
                        vx, vy, vyaw = 0.0, 0.0, 0.0
                    else:
                        self.is_settled = True

                if abs(x_w) < PRECISION_THRESH and abs(y_w) < PRECISION_THRESH:
                    vz = -0.05
                    vx, vy = 0.0, 0.0
                else:
                    vz = 0.0

            if not visible:
                vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

            self.send(vx, vy, vz, vyaw)

        # ── HUD ve Yayın (Ekrana değil, ROS 2 Ağına) ──
        color = (0, 255, 0) if locked else (0, 165, 255)
        cv2.putText(frame, f"STATE: {self.state} | MKR: {n_valid if visible else 0} | LCK: {locked}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, f"X_Err: {x_w:.2f}m | Y_Err: {y_w:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # ROS 2 Üzerinden Görüntüyü Yayınla
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_img.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Görüntü yayınlanırken hata: {e}")


def main():
    rclpy.init()
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()