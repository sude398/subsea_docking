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
        dt = np.clip(now - self.last_time, 0.01, 0.1)

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

    def update(self, z):
        dt = 0.04
        self.F[0,2] = dt
        self.F[1,3] = dt

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        z = np.array(z).reshape(2,1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x[0,0], self.x[1,0]