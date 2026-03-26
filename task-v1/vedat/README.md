# ⚓ RAMI ROV - Otonom ArUco Yerleşme (Docking) Sistemi

Bu paket, ROS 2 ve OpenCV kullanılarak su altı araçları (ROV/AUV) için geliştirilmiş, ArUco etiketleri tabanlı otonom yerleşme (docking) ve pozisyon koruma (station-keeping) sistemidir. TAC 2026 ve MATE ROV gibi yarışmalardaki otonom görevler için özel olarak tasarlanmıştır.

Sistem, kameradan aldığı görüntüleri işleyerek hedefin 6 Serbestlik Derecesindeki (6-DOF) konumunu hesaplar ve PID kontrolcüleri aracılığıyla MAVROS üzerinden iticilere (thrusters) RC Override komutları gönderir.

## ✨ Temel Özellikler
* **ArUco Tabanlı 6-DOF Konumlandırma:** `cv2.solvePnP` kullanılarak hedefin Z ekseni (mesafe) ve X/Y/Yaw sapmaları milimetrik olarak hesaplanır.
* **Gelişmiş Durum Makinesi (State Machine):** Sistem anlık duruma göre `SEARCHING`, `APPROACHING`, `PRECISION` ve `DOCKED` modları arasında otonom geçiş yapar.
* **Yumuşatılmış PID Kontrolü:** İticilerdeki osilasyonu (titremeyi) engellemek için Low-Pass Filter (Düşük Geçiren Filtre) ve "Ölü Bölge" (Deadband) toleransları içerir.
* **Dinamik HUD Ekranı:** Aracın anlık modunu, hedefe olan mesafesini, algılanan etiket sayısını ve motorlara giden PWM değerlerini canlı olarak ekrana yansıtır.
* **Sign Inversion Koruması:** Hedefe yaklaşırken veya hedefi geçerken ters yöne kaçma hataları matematiksel olarak düzeltilmiştir.

---

## 🏗️ Sistem Mimarisi (Nodelar)

Paket içerisinde birbirleriyle haberleşen iki temel ROS 2 Node'u bulunmaktadır:

### 1. `docking_mission_node` (Ana Kontrolcü)
Görüntü işlemeyi ve PID hesaplamalarını yapan ana beyindir.
* **Yayınladığı Topicler:** * `/mavros/rc/override` (MAVROS motor komutları)
  * `/target_status` (Anlık görev durumu)
* **Dinlediği Topicler:** * `/camera/image_raw` (Canlı kamera akışı - opsiyonel)
  * `/mavros/state` (Aracın Armed ve Guided durumları)

### 2. `mission_listener_node` (Görev Takipçisi)
Ana kontrolcüden gelen PWM değerlerini ve görev durumlarını dinleyerek terminalde loglar. Otonom yerleşme tamamlandığında (`DOCKING_COMPLETE`) robotik kol veya diğer sıralı görevleri tetiklemek için altyapı sağlar.

---

## ⚙️ Durum Makinesi (State Machine) Mantığı

1. **SEARCHING (Arama):** Kadrajda hiç etiket yoksa araç hedefe doğru kör uçuşla hafifçe ilerler (`Surge: 1600`).
2. **APPROACHING (Yaklaşma):** 1 veya 2 etiket algılandığında sistem diagonal merkez hesaplaması ile aracı X ve Y ekseninde kabaca merkeze çeker.
3. **PRECISION (Hassas Konumlandırma):** 3 veya daha fazla etiket algılandığında PnP algoritması devreye girer. Mesafe, yatay/dikey kayma ve açısal sapmalar PID ile sıfırlanmaya çalışılır.
4. **DOCKED (Yerleşme Tamamlandı):** Araç hedef mesafede (`0.45m`) belirtilen süre boyunca (örn: 10 saniye) stabil kalmayı başarırsa görev tamamlanır ve motorlar durdurulur (`PWM: 1500`).

---

## 🚀 Kurulum ve Çalıştırma

### Gereksinimler
* ROS 2 (Humble / Foxy)
* OpenCV ve `cv_bridge`
* MAVROS (`mavros_msgs`)
* Python 3.x ve `numpy`

### Derleme (Build)
ROS 2 workspace dizininize gidin ve paketi derleyin:
```bash
cd ~/your_ws
colcon build --packages-select aruco_docking
source install/setup.bash
```
### Çalıştırma

Ana görev node'unu (HUD ekranı ile birlikte) başlatmak için:
Bash
```bash
ros2 run aruco_docking docking_node
```

Motor komutlarını ve görev durumunu terminalden canlı takip etmek için (yeni bir terminalde):
```bash
source install/setup.bash
ros2 run aruco_docking mission_listener
```