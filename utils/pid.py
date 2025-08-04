from utils.filters import low_pass_filter

class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, max_output=2.0, min_output=-2.0, error_threshold=0.01, slowing_distance=10.0, slowing_minimum=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.max_output = max_output
        self.min_output = min_output

        self.error_threshold = error_threshold

        self.slowing_distance = slowing_distance
        self.slowing_minimum = slowing_minimum

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

    def compute(self, error, dt):
        v_min, v_max = self.min_output, self.max_output
        # Error için low-pass filtre uygulama
        error = low_pass_filter(error, self.prev_error, alpha=0.8)

        # Hedefe yaklaştıkça yavaşla
        distance = abs(error)
        if distance < self.slowing_distance:
            scale = distance / self.slowing_distance
            v_max = max(0.5, self.max_output * scale)
        # Hedef kabul edilebilecek eşikteyse 0 döndür ki çember çizme ve overshoot olmasın
        if distance < self.error_threshold:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # Çıktı için low-pass filtre uygulama
        raw_output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        filtered_output = low_pass_filter(raw_output, self.prev_output, alpha=0.85)
        output = max(self.slowing_minimum, filtered_output)

        # Hız sınırlanır
        output = max(v_min, min(v_max, output))

        self.prev_error = error
        return output
