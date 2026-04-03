import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import message_filters # Zaman eşlemesi için gerekli

class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')

        # 1. Abone olunacak konuları tanımla
        self.sub_img   = message_filters.Subscriber(self, Image, '/rov/image_processed')
        self.sub_state = message_filters.Subscriber(self, String, '/rov/state')
        self.sub_dist  = message_filters.Subscriber(self, Float32, '/rov/distance')

        # 2. Zaman eşitleyici (ApproximateTimeSynchronizer)
        # queue_size=10: 10 mesajlık kuyruk tutar
        # slop=0.1: Mesajlar arası 0.1 saniyelik gecikmeyi kabul eder
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_state, self.sub_dist],
            queue_size=10, slop=0.1
        )
        
        self.ts.registerCallback(self._common_callback)
        self.get_logger().info("Monitor Düğümü Başlatıldı: 3 konuya eş zamanlı abone olundu.")

    def _common_callback(self, img_msg, state_msg, dist_msg):
        # Tüm veriler aynı anda buraya gelir
        state = state_msg.data
        dist = dist_msg.data
        
        self.get_logger().info(f"Senkronize Veri -> Durum: {state} | Mesafe: {dist:.2f}m")

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