# İTÜ ROV Takımı - Ana Kamera Sistemi ve Kalibrasyon Durumu

Bu belge, İTÜ ROV 2026 aracı üzerindeki ana kamera sisteminin mevcut durumunu, veri akışını ve özellikle **otonom görevler (ArUco, PnP, SLAM vb.) için kritik olan kalibrasyon eksikliklerini** ve geçici kullanım yöntemlerini açıklar.

## 1. Görüntü Akışı ve Sistem Mimarisi

Kamera fiziksel olarak **Jetson Orin**'e bağlıdır. Görüntü işleme yükünü dağıtmak ve gecikmeyi (latency) düşürmek için H.264 formatında sıkıştırılarak GStreamer üzerinden ağa basılır. 

* **Yayıncı (Jetson Orin):** Görüntüyü `/dev/video0` üzerinden alır, UDP Port 5000 üzerinden yer istasyonuna fırlatır.
* **Alıcı Düğüm (ROS 2):** `rami_perception/gstreamer_camera_node.py` düğümü bu yayını yakalar ve ROS 2 sistemine dağıtır.
* **Kullanılacak Topic:** Görüntü işleme kodlarımız (Örn: ArUco tespiti) doğrudan **`/camera/image_raw`** topic'ine abone (subscribe) olmalıdır.

---

## 2. Mevcut Kalibrasyon Durumu (ÖNEMLİ UYARI!)

**Şu anki sistemde kameranın fiziksel (gerçek) kalibrasyon verileri EKSİKTİR.**

Sistemi hızlıca test edebilmek için `gstreamer_camera_node.py` dosyası içine **tahmini (approximate)** matematiksel değerler yazılmıştır. Sistem, kameranın mercek yapısını bilmediği için şu varsayımlarla çalışmaktadır:

* **Odak Uzaklığı ($f_x, f_y$):** Görüntü genişliğinin %80'i varsayılmıştır.
* **Optik Merkez ($c_x, c_y$):** Görüntünün tam piksel ortası varsayılmıştır.
* **Mercek Bozulması (Distortion - $d$):** Standart bir su altı bozulması varsayılarak `[0.1, -0.2, 0.0, 0.0, 0.0]` değerleri girilmiştir.

**Bunun Anlamı Nedir?**
Bu tahmini verilerle kameradan görüntü alınabilir ve 2 boyutlu basit renk/nesne tespiti yapılabilir. Ancak **6 Eksenli ArUco tespiti veya hedefe olan santimetre cinsinden uzaklık/derinlik hesaplamaları (PnP) hatalı sonuç verecektir.** Araç 1 metre uzaktaki bir hedefi 80 santim uzakta sanabilir.

---

## 3. Şu Anki Haliyle (Kalibrasyonsuz) Kodda Nasıl Kullanacağız?

Gerçek kalibrasyon yapılana kadar, yazacağımız 6 eksenli (6-DOF) ArUco veya konumlandırma kodlarında hata almamak için, **sistemin uydurduğu bu tahmini değerleri kodumuza manuel olarak girmeliyiz.** Kamerayı hangi çözünürlükte başlattığınıza göre aşağıdaki uygun matrisi ROS 2 Node'unuzun `__init__` kısmına eklemelisiniz:

### Seçenek 1: Sistem 640x480 Çözünürlükte Çalışıyorsa
```python
# DİKKAT: Bu değerler geçicidir (Approximate). 
# fx = 640 * 0.8 = 512.0
self.camera_matrix = np.array([
    [512.0, 0.0,   320.0],
    [0.0,   512.0, 240.0],
    [0.0,   0.0,   1.0]
], dtype=np.float32)

self.dist_coeffs = np.array([[0.1, -0.2, 0.0, 0.0, 0.0]], dtype=np.float32)
```

### Seçenek 2: Sistem 1920x1080 Çözünürlükte Çalışıyorsa
```python

# DİKKAT: Bu değerler geçicidir (Approximate).
# fx = 1920 * 0.8 = 1536.0
self.camera_matrix = np.array([
    [1536.0, 0.0,    960.0],
    [0.0,    1536.0, 540.0],
    [0.0,    0.0,    1.0]
], dtype=np.float32)

self.dist_coeffs = np.array([[0.1, -0.2, 0.0, 0.0, 0.0]], dtype=np.float32)
```
