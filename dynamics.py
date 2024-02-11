import numpy as np


class Dynamics:
    """
    :jr: Inertia Jacobian in scalar form
    :omega: Cubesat initial angular velocity
    :dt: time interval
    """

    def __init__(self, J: np.ndarray, omega: np.ndarray, dt: np.float64):
        self.J = J
        self.omega = omega
        self.dt = dt

    def step(self, angular_momentum: np.ndarray, angular_torque: np.ndarray, external_torque: np.ndarray):
        J_inv = np.linalg.inv(self.J)
        matrix_mult = np.dot(self.J, self.omega)
        cross1 = np.cross(self.omega, matrix_mult)
        cross2 = np.cross(self.omega, angular_momentum)
        addition = external_torque - cross1 - cross2 - angular_torque

        domega_dt = np.dot(J_inv, addition)
        # Integration
        self.omega += self.dt * domega_dt
