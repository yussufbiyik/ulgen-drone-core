class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, max_output=2.0, min_output=-2.0, error_threshold=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.max_output = max_output
        self.min_output = min_output

        self.error_threshold = error_threshold

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # Error için low-pass filtre uygulama
        alpha = 0.8
        error = alpha * self.prev_error + (1 - alpha) * error
        if abs(error) < self.error_threshold:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Hız sınırlanır
        output = max(self.min_output, min(self.max_output, output))

        self.prev_error = error
        return output
