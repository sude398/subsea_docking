import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Float32
import message_filters # zaman eşlemesi için

class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')

        # 1. Abone olunacak konuları tanımla
        self.sub_img   = message_filters.Subscriber(self, Image, '/rov/image_processed')
        self.sub_state = message_filters.Subscriber(self, String, '/rov/state')
        self.sub_dist  = message_filters.Subscriber(self, Float32, '/rov/distance')
        self.sub_vel   = message_filters.Subscriber(self, Float32MultiArray, '/rov/cmd_vel_debug')

        # 2. Zaman eşitleyiciye self.sub_vel'i de EKLE
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_state, self.sub_dist, self.sub_vel],
            queue_size=10, slop=0.1
        )
        
        self.ts.registerCallback(self._common_callback)
        self.get_logger().info("Monitor Düğümü Başlatıldı: 4 konuya eş zamanlı abone olundu.")

    # 3. Callback fonksiyonuna vel_msg parametresini EKLE
    def _common_callback(self, img_msg, state_msg, dist_msg, vel_msg):
        state = state_msg.data
        dist = dist_msg.data
        
        # Hız verilerini diziden ayıkla
        # vx: ileri, vy: sağ-sol, vz: dikey, vyaw: dönüş
        vx, vy, vz, vyaw = vel_msg.data
        
        # Log mesajını hızları içerecek şekilde güncelle
        self.get_logger().info(
            f"Durum: {state} | Mesafe: {dist:.2f}m | "
            f"Hızlar -> Vx:{vx:.2f}, Vy:{vy:.2f}, Vz:{vz:.2f}, Vyaw:{vyaw:.2f}"
        )

def main():
    rclpy.init()
    node = TaskMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
