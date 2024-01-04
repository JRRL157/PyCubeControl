import numpy as np


class Dynamics:
    """
    :jr: Inertia Jacobian in scalar form
    :omega: Cubesat initial angular velocity
    :dt: time interval
    """
    def __init__(self, jr: np.float64, omega: np.ndarray, dt: np.float64):
        self.jr = jr
        self.omega = omega
        self.dt = dt

    def step(self, input_velocity: np.ndarray, input_acceleration: np.ndarray):
        domega_dt = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        domega_dt[0] = self.jr * (
                self.omega[2] * input_velocity[1] - self.omega[1] * input_velocity[2] + input_acceleration[0])
        domega_dt[1] = self.jr * (
                self.omega[0] * input_velocity[2] - self.omega[2] * input_velocity[0] + input_acceleration[1])
        domega_dt[2] = self.jr * (
                self.omega[1] * input_velocity[0] - self.omega[0] * input_velocity[1] + input_acceleration[2])

        # Integration
        self.omega += self.dt * domega_dt
