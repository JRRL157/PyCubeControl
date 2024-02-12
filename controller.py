import numpy as np


class PID:

    def __init__(self, kp: np.float64, kd: np.float64, ki: np.float64, MAX_LIM: np.float64 = 12, dt: np.float64 = 1e-3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.MAX_LIM = MAX_LIM

        self.prev_error = np.array([0, 0, 0], dtype=np.float64)
        self.integral = np.array([0, 0, 0], dtype=np.float64)
        self.voltage = np.array([0, 0, 0], dtype=np.float64)

    def step(self, error: np.ndarray):
        proportional = self.kp * error

        self.integral += self.ki * error * self.dt
        derivative = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        self.voltage = proportional + self.integral + derivative
        self.voltage = np.array(
            [-self.MAX_LIM if x < -self.MAX_LIM else (self.MAX_LIM if x > self.MAX_LIM else x) for x in self.voltage])

    def get_output(self):
        return np.array([-12 if x <= -8192 else (12 if x >= 8192 else x / 8192.0) for x in self.voltage])


class BDot:
    def __init__(self, kp: np.float64 = 1.0, num: np.float64 = 100, area: np.float64 = 1, MAX_CURRENT: np.float64 = 1):
        self.kp = kp
        self.num = num
        self.area = area
        self.current = np.array([0, 0, 0], dtype=np.float64)
        self.MAX_CURRENT = MAX_CURRENT

    def update(self, error: np.ndarray, B):
        K = self.kp / np.power(np.linalg.norm(B), 2)
        cross_prod = np.cross(B, error)
        mult1 = K * cross_prod
        self.current = mult1 / (self.num * self.area)
        self.current = np.array([
            -self.MAX_CURRENT if x < -self.MAX_CURRENT else (self.MAX_CURRENT if x > self.MAX_CURRENT else x) for x in
            self.current])
