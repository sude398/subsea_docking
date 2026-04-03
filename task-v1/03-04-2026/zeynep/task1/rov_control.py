import time
import numpy as np
from pymavlink import mavutil
from std_msgs.msg import Float32MultiArray
from . import config


# ─────────────────────────────────────────────
# PID CONTROLLER
# ─────────────────────────────────────────────
class PID:
    
    #kp  → Araç hedefe çok yavaş yaklaşıyorsa artır.
    #ki  → Araç hedefe yakın ama küçük sabit sapma varsa artır.
    #kd  → Araç hedef etrafında sallanıyorsa artır.
    #rate_limit → Anlık güç sıçramalarını sınırlar.Araç çok sallanıyorsa düşür, yavaş tepki veriyorsa artır.

    def __init__(self, kp, ki, kd, limit, rate_limit=0.04):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.limit      = limit
        self.rate_limit = rate_limit
        self.reset()

    def update(self, e):
        now = time.time()
        dt  = now - self.last_time

        # Çok uzun aralık (örn. sistem donması) → integrali arttırmıyor
        if dt > 0.15:
            self.last_time = now
            return self.last_out

        dt = np.clip(dt, 0.01, 0.10)

        # Anti-windup: integral ±%30 limit ile kırpılır
        self.integral += e * dt
        self.integral  = np.clip(
            self.integral,
            -self.limit * 0.3,
            self.limit * 0.3
        )

        deriv = (e - self.last_e) / dt
        raw   = (self.kp * e) + (self.ki * self.integral) + (self.kd * deriv)
        raw   = np.clip(raw, -self.limit, self.limit)

        # Rate limiter: thruster'a ani güç değişimi verme
        delta = np.clip(raw - self.last_out, -self.rate_limit, self.rate_limit)
        out   = self.last_out + delta

        # Thruster deadband: çok küçük komutları sıfırla (%10 altı)
        # Motordan ses gelip pervane dönmüyorsa THRUSTER_DEADBAND'ı artır.
        if abs(out) < config.THRUSTER_DEADBAND:
            out = 0.0

        self.last_e    = e
        self.last_time = now
        self.last_out  = out
        return out

    def reset(self):
        self.integral  = 0.0
        self.last_e    = 0.0
        self.last_out  = 0.0
        self.last_time = time.time()


# ─────────────────────────────────────────────
# SMART SEARCH
# ─────────────────────────────────────────────
class SmartSearch:
    
    #Döngü: hafifçe aşağı in → sağa dön → sola dön → hafifçe yüksel -> tekrar.

    def __init__(self):
        self.reset()

    def reset(self):
        self.start_time = time.time()

    def get_velocity(self):
        """
        Döndürür: (vz, vyaw)
            vz   → dikey hız (negatif = aşağı)
            vyaw → yaw hız
        """
        t = time.time() - self.start_time

        if t < 3:
            # Hafifçe alçal — kamera açısı değişsin, platform görünebilir
            return -0.05, 0.0
        elif t < 7:
            # Sağa dön
            return 0.0, 0.18
        elif t < 11:
            # Sola dön
            return 0.0, -0.18
        elif t < 14:
            # Hafifçe yüksel — önceki alçalmayı telafi et
            return 0.05, 0.0

        self.reset()
        return 0.0, 0.0


# ─────────────────────────────────────────────
# MAVLink ARAÇ ARAYÜZÜ
# ─────────────────────────────────────────────
class RovInterface:

    def __init__(self):
        self.master = None
        self.armed  = False
        self._connect()

    def _connect(self):
        try:
            self.master = mavutil.mavlink_connection(
                config.SERIAL_PORT,
                baud=config.BAUD_RATE
            )
            self.master.wait_heartbeat(timeout=5)
            self.arm()
            print("[RovInterface] MAVLink bağlantısı başarılı.")
        except Exception as e:
            print(f"[RovInterface] MAVLink bağlantı hatası: {e}")
            self.master = None

    def arm(self):
        if not self.master:
            return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.armed = True
        print("[RovInterface] ROV ARM edildi.")

    def send_velocity(self, vx, vy, vz, vyaw):
        """
        MAVLink MANUAL_CONTROL paketi gönderir.

        Eksen tanımları (ArduSub varsayılan):
            vx   > 0 → ileri
            vy   > 0 → sağa
            vz   < 0 → aşağı   (MAVLink'te z=500 hover, <500 aşağı, >500 yukarı)
            vyaw > 0 → saat yönünde dönüş
        """
        if not self.master or not self.armed:
            return

        x = int(np.clip(vx   * 1000, -1000, 1000))
        y = int(np.clip(vy   * 1000, -1000, 1000))
        z = int(np.clip(500  + vz * 500, 0, 1000))
        r = int(np.clip(vyaw * 1000, -1000, 1000))

        self.master.mav.manual_control_send(
            self.master.target_system,
            x, y, z, r, 0
        )

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)import time
import numpy as np
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from . import config


# ───────────────
# PID CONTROLLER 
# ───────────────
class PID:
    def __init__(self, kp, ki, kd, limit, rate_limit=0.04):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.rate_limit = rate_limit
        self.reset()

    def update(self, e):
        now = time.time()
        dt = now - self.last_time
        dt = np.clip(dt, 0.01, 0.05)  

        # Integral birikmesini önler (anti-windup)
        self.integral += e * dt
        self.integral = np.clip(
            self.integral,
            -self.limit * 0.3,
            self.limit * 0.3
        )

        deriv = (e - self.last_e) / dt

        raw = (self.kp * e) + (self.ki * self.integral) + (self.kd * deriv)
        raw = np.clip(raw, -self.limit, self.limit)

        # RATE LIMITER
        # yeni güç değeri öncekinden çok farklıysa thrustera aniden güç verme yavaş yavaş ver
        # araç çok sallanıyorsa rate limit değerini düşür
        # araç çok yavaş yaklaşıyorsa rate limit değerini arttır 
        delta = np.clip(raw - self.last_out, -self.rate_limit, self.rate_limit)
        out = self.last_out + delta

        # Thruster deadband
        # güç %10dan azsa thruster hareket etmesin
        # motordan ses gelip pervane dönmüyorsa değeri arttır
        if abs(out) < config.THRUSTER_DEADBAND:
            out = 0.0

        self.last_e = e
        self.last_time = now
        self.last_out = out

        return out

    def reset(self):
        self.integral = 0.0
        self.last_e = 0.0
        self.last_out = 0.0
        self.last_time = time.time()


class SmartSearch:
    def __init__(self):
        self.reset()

    def reset(self):
        self.start_time = time.time()

    def get_velocity(self):
        t = time.time() - self.start_time

        if t < 3:
            return 0.05, 0.0      # 3sn yavaşça yüksel
        elif t < 7:
            return 0.0, 0.18      # 4sn yavaşça sağ
        elif t < 11:
            return 0.0, -0.18     # 4sn yavaşça sol

        self.reset()
        return 0.0, 0.0


# ─────────────────────────────────────────────
# MAVLINK INTERFACE
# ─────────────────────────────────────────────
class RovInterface(Node):

    def __init__(self):
        super().__init__('rov_interface')

        self.master = None
        self.armed = False

        try:
            self.master = mavutil.mavlink_connection(
                config.SERIAL_PORT,
                baud=config.BAUD_RATE
            )

            self.get_logger().info("MAVLink bekleniyor...")
            self.master.wait_heartbeat(timeout=5)

            self.get_logger().info("Bağlantı başarılı!")
            self.arm()

        except Exception as e:
            self.get_logger().error(f"MAVLink hata: {e}")
        # hız komutları mavlink manual control send paketi ile gönderilir
        # hangi komutların gittiği cmd_vel_debug topic'undan izlenebilir
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/rov/cmd_vel_debug',
            10
        )

    def arm(self):
        if not self.master:
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

        self.armed = True
        self.get_logger().info("ARM edildi")

    def send_velocity(self, vx, vy, vz, vyaw):

        if not self.master or not self.armed:
            return

        x = int(np.clip(vx * 1000, -1000, 1000))
        y = int(np.clip(vy * 1000, -1000, 1000))
        z = int(np.clip(500 + vz * 500, 0, 1000))
        r = int(np.clip(vyaw * 1000, -1000, 1000))

        self.master.mav.manual_control_send(
            self.master.target_system,
            x, y, z, r, 0
        )

        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(vz), float(vyaw)]
        self.pub.publish(msg)

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
