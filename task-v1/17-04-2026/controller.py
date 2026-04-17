import time
import numpy as np


# ─────────────────────────────────────────────
# PID
# ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.last_e = 0.0
        self.last_out = 0.0
        self.last_time = time.time()
        self.integral_limit = limit * 0.5
        self.rate_limit = 0.05
        self.deadband = 0.02
        self.d_filter = 0.7
        self.d_last = 0.0

    def update(self, e):
        now = time.time()

        # Küçük hatada kontrolü devre dışı bırakır ve son çıkışı yumuşak şekilde azaltır (deadband + smoothing)
        if abs(e) < 0.01:
            self.last_out *= 0.5  # Hızı her döngüde yarıya düşür (süzülme)
            self.last_e = e
            self.last_time = now
            return self.last_out  # süzülen hızı gönder

        #dt = np.clip(now - self.last_time, 0.01, 0.1)
        dt = np.clip(now - self.last_time, 0.02, 0.08) # daha stabil derivative ve rov için daha gerçekçi


        self.integral += e * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        d_raw = (e - self.last_e) / dt
        d = self.d_filter * self.d_last + (1 - self.d_filter) * d_raw
        self.d_last = d

        out = self.kp*e + self.ki*self.integral + self.kd*d
        out = np.clip(out, -self.limit, self.limit)

        delta = np.clip(out - self.last_out, -self.rate_limit, self.rate_limit)
        out = self.last_out + delta

        if abs(out) < self.deadband:
            out = 0.0

        self.last_out = out
        self.last_e = e
        self.last_time = now

        
        return out


# ─────────────────────────────────────────────
# KALMAN FILTER (XY)
# ─────────────────────────────────────────────
class Kalman2D:
    def __init__(self):
        self.x = np.zeros((4,1))
        self.P = np.eye(4) * 0.5
        self.F = np.eye(4)
        self.H = np.array([[1,0,0,0], [0,1,0,0]])
        self.Q = np.eye(4) * 0.02
        self.R = np.eye(2) * 0.05

        self.R_base = np.eye(2) * 0.05  # EKLENDİ (adaptive R için base noise)
        self.last_time = time.time()    # EKLENDİ (dynamic dt için zaman takibi)
        self.initialized = False        # EKLENDİ (ilk ölçüm stabil başlatma)

    def update(self, z):
        # dt = 0.04 #ESKİ HAL: sabit zaman adımı kullanılıyordu (FPS bağımlıydı)
        now = time.time() # alttaki için eklendi
        dt = np.clip(now - self.last_time, 0.01, 0.1) # YENİ HAL: gerçek zaman farkı kullanılıyor → daha stabil ve gerçekçi
        self.last_time = now # eklendi

        self.F[0,2] = dt
        self.F[1,3] = dt

        # EKLENDİ: ilk ölçümde Kalman state direkt başlatılıyor
        if not self.initialized:
            self.x[0,0], self.x[1,0] = z
            self.initialized = True
            return z[0], z[1]

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        #EKLENDİ: measurement büyüklüğüne göre noise artırılıyor (uzak/noisy ölçüm daha az güvenilir)
        distance = np.linalg.norm(z)
        self.R = self.R_base * (1 + 0.8 * distance)

        z = np.array(z).reshape(2,1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        # K = self.P @ self.H @ self.H.T.T @ np.linalg.inv(S) #Eski hal
        K = self.P @ self.H.T @ np.linalg.inv(S) #YENİ DOĞRU HAL

        # EKLENDİ: outlier rejection (çok büyük hata varsa update yapılmaz)
        if np.linalg.norm(y) > 1.5:
            return self.x[0,0], self.x[1,0]

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x[0,0], self.x[1,0]