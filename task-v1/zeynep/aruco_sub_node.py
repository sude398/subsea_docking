import time
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from std_msgs.msg import Bool, String, Float32

# Log hızlarını sınırlamak için (Terminalin donmaması için)
CMD_RATE_LIMIT    = 0.5   # Komutları 2 saniyede bir bas
ERROR_RATE_LIMIT  = 0.5   # Hataları 2 saniyede bir bas
STATUS_RATE_LIMIT = 2.0   # Durum mesajları aralığı

class ArucoSubscriberNode(Node):
    def __init__(self):
        super().__init__('aruco_subscriber')

        # ── Durum Takibi ──────────────────────────────────────────────────────
        self.last_status_text  = ""
        self.last_status_time  = 0.0
        self.last_lock_state   = None
        self.last_cmd_time     = 0.0
        self.last_error_time   = 0.0

        # ── Subscriber'lar ────────────────────────────────────────────────────
        # Ana düğümden (Node Cam) gelen verileri dinliyoruz
        self.create_subscription(PoseStamped,  '/aruco/pose',           self.cb_pose,     10)
        self.create_subscription(Float32,      '/aruco/distance',       self.cb_distance, 10)
        self.create_subscription(Bool,         '/aruco/locked',         self.cb_locked,   10)
        self.create_subscription(String,       '/aruco/status',         self.cb_status,   10)
        self.create_subscription(Point,        '/aruco/target_center',  self.cb_error,    10) # Piksel hatası buraya geliyor
        
        # MAVROS'a giden hız komutlarını dinliyoruz
        self.create_subscription(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.cb_cmd, 10)

        self.get_logger().info('🚀 ArUco Subscriber (Gelişmiş İzleyici) Başlatıldı.')
        self.get_logger().info('──────────────────────────────────────────────────')
        self.get_logger().info('📡 DİNLENEN TÜM TOPICLER:')
        self.get_logger().info('  1. /aruco/target_center  -> [Point] Piksel Sapması (X, Y)')
        self.get_logger().info('  2. /aruco/distance       -> [Float32] Metre Cinsinden Mesafe')
        self.get_logger().info('  3. /aruco/locked         -> [Bool] Kilitlenme/Kayıp Durumu')
        self.get_logger().info('  4. /aruco/pose           -> [PoseStamped] 3D Pozisyon ve Açı (Yaw)')
        self.get_logger().info('  5. /aruco/status         -> [String] Sistem Durum Mesajları')
        self.get_logger().info('  6. /mavros/setpoint_velocity/cmd_vel -> [TwistStamped] Motor Komutları')
        self.get_logger().info('──────────────────────────────────────────────────')
    # ── CALLBACKS ─────────────────────────────────────────────────────────────

    def cb_error(self, msg: Point):
        """Piksel hatalarını ve yönleri analiz eder."""
        now = time.time()
        if (now - self.last_error_time) >= ERROR_RATE_LIMIT:
            ex = int(msg.x - 960) # Merkezden sapma
            ey = int(msg.y - 540)
            
            # Yönlendirme mantığı (Aşağı bakış kamerasına göre)
            dir_x = "SAĞDA →" if ex > 20 else "SOLDA ←" if ex < -20 else "MERKEZ"
            dir_y = "GERİDE ↓" if ey > 20 else "İLERİDE ↑" if ey < -20 else "MERKEZ"
            
            self.get_logger().info(f'[TARGET] Sapma X: {ex:4} ({dir_x}) | Sapma Y: {ey:4} ({dir_y})')
            self.last_error_time = now

    def cb_cmd(self, msg: TwistStamped):
        """Motora giden gerçek hız komutlarını gösterir."""
        now = time.time()
        vx  = msg.twist.linear.x
        vy  = msg.twist.linear.y
        vz  = msg.twist.linear.z
        v_yaw = msg.twist.angular.z

        if (now - self.last_cmd_time) >= CMD_RATE_LIMIT:
            # Sadece hareket varsa bas (Gereksiz 0.00 loglarını engellemek için)
            if any(abs(v) > 0.01 for v in [vx, vy, vz, v_yaw]):
                self.get_logger().info(
                    f'[MOTOR]  SURGE:{vx:+.2f} | SWAY:{vy:+.2f} | HEAVE:{vz:+.2f} | YAW:{v_yaw:+.2f}'
                )
                self.last_cmd_time = now

    def cb_distance(self, msg: Float32):
        """Mesafe bilgisi."""
        if msg.data < 0.5:
            self.get_logger().warn(f'[DIST]   KRİTİK YAKINLIK: {msg.data:.3f}m')
        else:
            self.get_logger().info(f'[DIST]   Mesafe: {msg.data:.3f}m')

    def cb_locked(self, msg: Bool):
        """Kilitlenme durumu değiştiğinde bildir."""
        if msg.data != self.last_lock_state:
            self.last_lock_state = msg.data
            status = "✅ KİLİTLENDİ (Hedef Görünüyor)" if msg.data else "❌ KAYIP (Arama Modu)"
            self.get_logger().info(f'[LOCK]   {status}')

    def cb_pose(self, msg: PoseStamped):
        """Opsiyonel: Detaylı Pozisyon Verisi."""
        q = msg.pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        yaw, _, _ = r.as_euler('ZYX', degrees=True)
        self.get_logger().info(f'[POSE]   Açı (Yaw): {yaw:+.1f}°')

    def cb_status(self, msg: String):
        """Sistemden gelen metin mesajları."""
        now = time.time()
        if msg.data != self.last_status_text or (now - self.last_status_time) > STATUS_RATE_LIMIT:
            self.get_logger().info(f'[STATUS] {msg.data}')
            self.last_status_text = msg.data
            self.last_status_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Kullanıcı durdurdu.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()