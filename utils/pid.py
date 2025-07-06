import numpy as np

class PID:
    def __init__(self, 
                    Kp = [0.8, 0.8, 0.5], 
                    Ki = [0.02, 0.02, 0.01], 
                    Kd = [0.3, 0.3, 0.2], 
                    dt = 0.05
                ):
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)

    def compute(self, target, current):
        """"
        PID kontrol algoritması
        """
        e = target - current
        
        # İntegral ve türev bileşenleri
        self.integral += e * self.dt
        derivative = (e - self.prev_error) / self.dt
        
        self.prev_error = e
        velocity = self.Kp*e + self.Ki*self.integral + self.Kd*derivative
        # Hız limitleri
        clipped_velocity = np.clip(velocity, -2.0, 2.0)
        
        return clipped_velocity