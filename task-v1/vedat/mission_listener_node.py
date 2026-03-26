#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String

class ROVMissionListenerNode(Node):
    def __init__(self):
        super().__init__('rov_mission_listener_node')
        
        # --- ABONELİKLER (SUBSCRIPTIONS) ---
        # 1. PWM Komutlarını Dinle
        self.rc_sub = self.create_subscription(
            OverrideRCIn,
            '/mavros/rc/override',
            self.rc_callback,
            10
        )
        
        # 2. Görev Durumunu Dinle
        self.status_sub = self.create_subscription(
            String,
            '/target_status',
            self.status_callback,
            10
        )
        
        # Durum takibi için değişken
        self.is_docking_finished = False
        
        self.get_logger().info("🎧 ROV Görev Dinleyici Node başlatıldı. Veriler bekleniyor...")

    def rc_callback(self, msg: OverrideRCIn):
        """
        Otonom sistemden gelen PWM değerlerini yakalar.
        """
        # MAVROS RC Override mesajında indeksler 0'dan başlar.
        # Bizim sistemimizde:
        # CH3 (Heave) -> index 2
        # CH4 (Yaw)   -> index 3
        # CH5 (Surge) -> index 4
        # CH6 (Sway)  -> index 5
        
        heave = msg.channels[2]
        yaw   = msg.channels[3]
        surge = msg.channels[4]
        sway  = msg.channels[5]
        
        # 65535 değeri, MAVROS'ta "bu kanala dokunma/boş bırak" anlamına gelir.
        # 0 değeri ise genelde nötr veya sistemin kapalı olduğunu gösterir.
        # Sadece aktif otonom komutları filtreleyip ekrana basalım:
        if surge not in (0, 65535):
            self.get_logger().info(
                f"🚤 [İTİCİ KOMUTU] Surge: {surge} | Sway: {sway} | Heave: {heave} | Yaw: {yaw}"
            )

    def status_callback(self, msg: String):
        """
        Görsel işleme ve yerleşme (docking) durumunu yakalar.
        """
        status = msg.data
        
        if status == "DOCKING_COMPLETE" and not self.is_docking_finished:
            self.get_logger().info("🎯 [GÖREV BAŞARILI]: Otonom yerleşme tamamlandı!")
            self.is_docking_finished = True
            
            # --- BURAYA SONRAKİ GÖREV TETİKLEYİCİLERİ EKLENEBİLİR ---
            # Örnek:
            # self.trigger_manipulator() 
            # self.switch_to_manual_mode()
            
        elif status != "DOCKING_COMPLETE":
            # Gerekirse SEARCHING, APPROACHING gibi diğer durumları da buradan takip edebilirsin.
            self.get_logger().debug(f"ℹ️ [DURUM]: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = ROVMissionListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Dinleyici Node kapatılıyor...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()