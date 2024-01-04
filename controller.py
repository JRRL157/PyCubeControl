import numpy as np


class PID:

    def __init__(self, kp: np.float64, kd: np.float64, ki: np.float64, dt: np.float64):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.prev_error = 0
        self.integral = 0
        self.output = 0

    def step(self, error: np.ndarray):
        proportional = self.kp * error

        self.integral += self.ki * error * self.dt
        derivative = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        self.output = proportional + self.integral + derivative

    def get_output(self):
        return np.array([-1 if x <= -8192 else (1 if x >= 8192 else x / 8192.0) for x in self.output])
