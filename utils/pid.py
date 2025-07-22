class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, max_output=2.0, min_output=-2.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.max_output = max_output
        self.min_output = min_output

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Clamp the output
        output = max(self.min_output, min(self.max_output, output))

        self.prev_error = error
        return output
