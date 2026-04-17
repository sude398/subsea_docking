import time
import numpy as np
from pymavlink import mavutil

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.integral_limit = limit * 0.5

    def update(self, error):
        now = time.time()
        dt = max(now - self.last_time, 0.01)
        self.integral = np.clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        derivative = (error - self.last_error) / dt
        out = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        self.last_time = now
        return np.clip(out, -self.limit, self.limit)

class ServoController:
    def __init__(self, mav_master, config):
        self.master = mav_master
        self.cfg = config
        self.current_angle = 0.0 # Dahili değişken
        self.pid = PID(0.06, 0.01, 0.005, 15.0) 

    def update_by_error(self, y_error_px):
        adjustment = self.pid.update(y_error_px)
        self.current_angle = np.clip(self.current_angle + adjustment, 0, self.cfg.SERVO_ANGLE_MAX)
        
        ratio = self.current_angle / self.cfg.SERVO_ANGLE_MAX
        pwm = int(self.cfg.SERVO_PWM_MIN + ratio * (self.cfg.SERVO_PWM_MAX - self.cfg.SERVO_PWM_MIN))
        
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, self.cfg.CAMERA_SERVO_NUM, pwm, 0, 0, 0, 0, 0
            )

    def reset(self):
        self.current_angle = 0.0
        self.pid.integral = 0.0
        self.update_by_error(0)

    @property
    def angle(self):
        """Dışarıdan çağrılırken 'self.servo.angle' olarak kullanılır."""
        return self.current_angle