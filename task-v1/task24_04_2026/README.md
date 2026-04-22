ROV Otonom Kenetlenme ve İniş Sistemi (Autonomous Docking)

Bu proje, bir ROV'un (Remotely Operated Vehicle) ArUco markerları kullanarak su altındaki bir platformu tespit etmesini, kendini bu platforma göre hizalamasını ve tam merkeze otonom iniş yapmasını sağlayan bir kontrol yazılımıdır. Sistem; bilgisayarlı görü (Computer Vision), olasılıksal tahminleme (Kalman Filter) ve kontrol teorisini (PID) birleştirerek kararlı bir çalışma sunar.
🚀 Öne Çıkan Teknik Özellikler

    Gelişmiş Vizyon Modeli: Su altı optik etkilerini (kırılma indisi: 1.33) ve kameranın eğim açısını (pitch: 45°) kompanse eden trigonometrik dünya koordinatı hesaplaması.

    Adaptif Kalman Filtresi: Ölçüm gürültüsünü mesafeye göre dinamik olarak ayarlayan ve aykırı değerleri (outlier rejection) reddeden 2D Kalman filtresi.

    Hafızalı Onay Mekanizması: İniş öncesi 4 marker'ın belirli bir süre (CONFIDENCE_THRESHOLD) boyunca doğrulanmasıyla sağlanan güvenli kilit.

    Hata Sönümlemeli PID: Hedefteki küçük sapmalarda motorları aniden durdurmak yerine hızı yumuşak bir şekilde azaltan (smoothing) ve deadband içeren PID kontrolü.

    Zamanlayıcılı Aktif Arama: Hedef kaybedildiğinde ilk 5 saniye ileri gidiş ve ardından kendi ekseninde tarama yapan stratejik arama algoritması.

🔄 FSM (Sonlu Durum Makinesi) Akışı

    SEARCHING: Marker arama aşaması. Hedef 10 saniyeden uzun süre kayıp değilse "onaylı" durumunu korur.

    ALIGNING: Hedef bulunduğunda merkez hattına ve paralel açıya hizalanma.

    APPROACHING: Platformun üzerine yaklaşma. 4 marker onayı ve kamera ofset kontrolü (CAMERA_OFFSET_Y) burada yapılır.

    DESCENDING: Yatay hareketi kesip tam merkezde dikey inişe geçiş (vz: -0.15).

    FINAL_ALIGN: Zemine çok yakınken (0.45m) başlayan hassas ve yavaş alçalma (vz: -0.08).

    DOCKED: Kenetlenme tamamlandığında motorların durdurulması.

⚠️ Güvenlik ve Hata Yönetimi

    Dinamik Görüş Kaybı Yönetimi: İniş (DESCENDING) aşamasında markerlar görüş alanından çıksa dahi araç durmaz. Sistem, son bilinen hız verilerini %80 oranında sönümleyerek (last_vx * 0.8) uygulamaya devam eder. Bu, aracın ataletiyle hedefe güvenli bir şekilde süzülmesini sağlar.

    10 Saniye Hafıza Toleransı: SEARCHING moduna geçildiğinde, 4 marker onayı (confirmed_4_markers) hemen sıfırlanmaz. Eğer marker kaybı 10 saniyeden kısa sürerse, araç onaylı durumunu korur. Bu süre, inişin tamamlanması için gereken süreden fazladır ve anlık görüntü kayıplarında sistemin kararlılığını garanti eder.

    Akıllı Reset Protokolü: Hedef 10 saniye boyunca bulunamazsa, sistem tüm onayları sıfırlayarak güvenli bir şekilde aktif arama prosedürüne geri döner.

🛠 Donanım ve Yazılım

    Donanım: Jetson Orin Nano / Pixhawk (Mavlink) / USB Kamera.

    Yazılım: ROS 2, OpenCV, PyMavlink, NumPy.

⚙️ Kurulum

    config.py içerisindeki CAMERA_MATRIX ve PITCH_ANGLE_RAD değerlerini kalibre edin.

    CAMERA_OFFSET_Y değerini, kameranızın aracın fiziksel merkezine olan uzaklığına göre ayarlayın.