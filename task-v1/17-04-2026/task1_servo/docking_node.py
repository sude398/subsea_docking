import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pymavlink import mavutil
import config as cfg
from controller import PID, ServoController
from vision import Vision
import builtins

# ROS 2 Mesaj Tipleri
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image # Görüntü yayını için eklendi
from cv_bridge import CvBridge    # OpenCV -> ROS dönüşümü için eklendi

class DockingNode(Node):
    def __init__(self):
        builtins.super().__init__('docking_node')
        
        # 1. Donanım ve İletişim Kurulumu
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMAGE_HEIGHT)
        
        self.master = mavutil.mavlink_connection(cfg.SERIAL_PORT, baud=cfg.BAUD_RATE)
        self.master.wait_heartbeat()
        self._set_mode("ALT_HOLD") 
        
        # 2. Modüller ve Köprüler
        self.vision = Vision()
        self.servo = ServoController(self.master, cfg)
        self.bridge = CvBridge() # CvBridge başlatıldı
        
        # 3. Kontrolcüler
        self.pid_lat = PID(1.5, 0.1, 0.3, cfg.MAX_LATERAL)
        self.pid_fwd = PID(1.2, 0.1, 0.2, cfg.MAX_FORWARD)
        self.pid_yaw = PID(0.8, 0.05, 0.1, cfg.MAX_YAW)
        
        # 4. ROS 2 Yayıncıları (Publisher)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.telemetry_pub = self.create_publisher(Float32MultiArray, '/docking/telemetry', 10)
        # Görüntü yayıncısı eklendi
        self.image_pub = self.create_publisher(Image, '/docking/processed_frame', 10)
        
        self.state = "SEARCHING"
        self.timer = self.create_timer(0.04, self.loop) # 25 FPS

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Görüntü İşleme ve Servo Takibi
        visible, locked, x_w, y_w, z_w, yaw, frame, y_err_px = self.vision.process(frame, self.servo.current_angle)
        
        if visible:
            self.servo.update_by_error(y_err_px)
            # Hareket mantığı (FSM)
            vx = self.pid_lat.update(x_w)
            vy = self.pid_fwd.update(y_w)
            vyaw = self.pid_yaw.update(yaw)
            vz = self._calculate_dynamic_vz(z_w)
            
            if z_w < cfg.DIST_DOCKED: self.state = "DOCKED"
            elif z_w < cfg.DIST_DESCENDING: self.state = "DESCENDING"
            else: self.state = "ALIGNING"
        else:
            self.state = "SEARCHING"
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0

        # MAVLINK Komut Gönderimi
        self.master.mav.manual_control_send(
            self.master.target_system,
            builtins.int(vx * 1000), builtins.int(vy * 1000), builtins.int(500 + (vz * 1000)), builtins.int(vyaw * 1000), 0
        )

        # 3. ÇÖZÜM: Görüntüyü ROS Üzerinden Yayınla
        try:
            # İşlenmiş frame'i ROS mesajına çevir ve gönder
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except builtins.Exception as e:
            self.get_logger().error(f"Görüntü yayınlanamadı: {e}")


    def _calculate_dynamic_vz(self, z_w):
        if z_w > cfg.DIST_DESCENDING: return 0.0
        ratio = np.clip(z_w / cfg.DIST_DESCENDING, 0, 1)
        return cfg.VZ_MIN + (cfg.VZ_MAX - cfg.VZ_MIN) * ratio

    def _set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(self.master.target_system, 
                                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
                                     mode_id)


def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except builtins.KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()