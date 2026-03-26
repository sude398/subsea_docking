# 🤿 RAMI ROV — Basit Kontrol Rehberi

## 6 Eksen mi, 3 Eksen mi?

Kısa cevap: **Aslında 3+3 = 6 eksen**, ama pratikte **4 tanesi yeterli**.

```
  3 ÖTELEME (bir yere gitmek)     3 DÖNME (yön değiştirmek)
  ─────────────────────────────   ─────────────────────────────
  Surge  = İleri / Geri           Yaw   = Sağa / Sola dönme
  Sway   = Sağa / Sola kayma     Pitch = Burnu yukarı/aşağı
  Heave  = Yukarı / Aşağı        Roll  = Yana yatma
```

**Öteleme** = ROV'un yerini değiştirir (A noktasından B'ye gider)
**Dönme** = ROV'un yönünü değiştirir (yerinde durur ama bakış açısı değişir)

> [!NOTE]
> Günlük kullanımda **Surge + Sway + Heave + Yaw** (4 eksen) yeterli. Pitch ve Roll genelde otopilot (STABILIZE modu) tarafından otomatik dengelenir. Sadece 8 motorlu custom frame'de elle kontrol edilir.

---

## Nasıl Komut Göndeririz?

ROV'a komut göndermek için `/mavros/rc/override` topic'ine PWM değerleri yayınlıyoruz.

**Kurallar:**
- Her kanal **1100–1900** arası değer alır
- **1500** = dur (nötr)
- **1500'den büyük** = pozitif yön (ileri, sağa, yukarı, sağa dön)
- **1500'den küçük** = negatif yön (geri, sola, aşağı, sola dön)

```
channels: [Pitch, Roll, Heave, Yaw, Surge, Sway, ...]
             ch1    ch2   ch3    ch4   ch5    ch6
```

---

## Terminal ile Test Komutları

### İleri Git
```bash
ros2 topic pub --rate 25 /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1500,1500,1800,1500, 0,0,0,0,0,0,0,0,0,0,0,0]}"
#                                     ^^^^
#                                  Ch5=1800 → ileri
```

### Geri Git
```bash
ros2 topic pub --rate 25 /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1500,1500,1200,1500, 0,0,0,0,0,0,0,0,0,0,0,0]}"
#                                     ^^^^
#                                  Ch5=1200 → geri
```

### Sağa Kay
```bash
ros2 topic pub --rate 25 /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1500,1500,1500,1800, 0,0,0,0,0,0,0,0,0,0,0,0]}"
#                                          ^^^^
#                                       Ch6=1800 → sağa
```

### Yukarı Çık
```bash
ros2 topic pub --rate 25 /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1800,1500,1500,1500, 0,0,0,0,0,0,0,0,0,0,0,0]}"
#                       ^^^^
#                    Ch3=1800 → yukarı
```

### Sağa Dön (Yaw)
```bash
ros2 topic pub --rate 25 /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1500,1800,1500,1500, 0,0,0,0,0,0,0,0,0,0,0,0]}"
#                            ^^^^
#                         Ch4=1800 → sağa dön
```

### Dur
```bash
ros2 topic pub --once /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500,1500,1500,1500,1500,1500, 0,0,0,0,0,0,0,0,0,0,0,0]}"
```

> [!IMPORTANT]
> `--rate 25` = saniyede 25 mesaj gönderir (sürekli hareket). `--once` = tek mesaj gönderir (test için). ArduSub ~1 sn mesaj gelmezse motorları durdurur.

---

## Hızlı Referans Tablosu

| Ne yapmak istiyorsun? | Hangi kanal? | Değer |
|----------------------|-------------|-------|
| İleri | ch5 `[4]` | 1500↑ (örn: 1800) |
| Geri | ch5 `[4]` | 1500↓ (örn: 1200) |
| Sağa kay | ch6 `[5]` | 1500↑ |
| Sola kay | ch6 `[5]` | 1500↓ |
| Yukarı | ch3 `[2]` | 1500↑ |
| Aşağı | ch3 `[2]` | 1500↓ |
| Sağa dön | ch4 `[3]` | 1500↑ |
| Sola dön | ch4 `[3]` | 1500↓ |

**1500'e ne kadar uzaklaşırsan, o kadar hızlı gidersin.** 1900 = tam gaz, 1100 = tam ters.

---

## Python ile Kontrol

### Örnek 1 — Basit Sürekli Komut

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class ROVKontrol(Node):
    def __init__(self):
        super().__init__('rov_test')
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.timer = self.create_timer(0.04, self.gonder)  # 25 Hz

    def gonder(self):
        rc = OverrideRCIn()
        rc.channels = [1500] * 18  # hepsi nötr

        # İstediğin kanalı değiştir:
        rc.channels[4] = 1700  # ch5 = %50 ileri
        rc.channels[3] = 1600  # ch4 = hafif sağa dön

        self.pub.publish(rc)

def main():
    rclpy.init()
    rclpy.spin(ROVKontrol())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Örnek 2 — Sıralı Test (Her Yönde 2 Saniye)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
import time

class ROVKontrol(Node):
    def __init__(self):
        super().__init__('rov_control_demo')
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.channels = [1500] * 18  # başlangıçta tüm kanallar nötr

    def send_rc(self, surge=1500, sway=1500, heave=1500, yaw=1500, duration=1.0):
        """RC komutu gönder ve belirtilen süre bekle"""
        msg = OverrideRCIn()
        msg.channels = self.channels.copy()
        msg.channels[4] = surge  # ch5 → ileri/geri
        msg.channels[5] = sway   # ch6 → sağa/sola kayma
        msg.channels[2] = heave  # ch3 → yukarı/aşağı
        msg.channels[3] = yaw    # ch4 → sağa/sola dön

        end_time = time.time() + duration
        while time.time() < end_time:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.04)  # 25 Hz
        # Süre bitince motorları nötre al
        self.stop_motors()

    def stop_motors(self):
        msg = OverrideRCIn()
        msg.channels = [1500] * 18
        for _ in range(5):  # birkaç kez gönder, kaybolma ihtimaline karşı
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.04)

def main():
    rclpy.init()
    node = ROVKontrol()

    try:
        node.get_logger().info("ROV Test Başlıyor: Her yönde 2 saniye hareket")

        # 1️⃣ İleri → Surge ±200 PWM (%50 güç)
        node.get_logger().info("İleri")
        node.send_rc(surge=1700, duration=2)

        # 2️⃣ Geri
        node.get_logger().info("Geri")
        node.send_rc(surge=1300, duration=2)

        # 3️⃣ Sağa kay
        node.get_logger().info("Sağa kay")
        node.send_rc(sway=1700, duration=2)

        # 4️⃣ Sola kay
        node.get_logger().info("Sola kay")
        node.send_rc(sway=1300, duration=2)

        # 5️⃣ Yukarı
        node.get_logger().info("Yukarı")
        node.send_rc(heave=1700, duration=2)

        # 6️⃣ Aşağı
        node.get_logger().info("Aşağı")
        node.send_rc(heave=1300, duration=2)

        # 7️⃣ Sağa dön
        node.get_logger().info("Sağa dön")
        node.send_rc(yaw=1600, duration=2)

        # 8️⃣ Sola dön
        node.get_logger().info("Sola dön")
        node.send_rc(yaw=1400, duration=2)

        node.get_logger().info("Tüm testler tamamlandı. Motorlar durdu.")

    except KeyboardInterrupt:
        node.stop_motors()
        node.get_logger().info("Test durduruldu, motorlar nötre alındı.")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> [!CAUTION]
> Araç önce **ARM** edilmeli, yoksa motorlar çalışmaz. Mesajları sürekli (25 Hz) göndermelisin.
