import numpy as np


class Dynamics:
    """
    :jr: Inertia Jacobian in scalar form
    :omega: Cubesat initial angular velocity
    :dt: time interval
    """

    def __init__(self, J,omega,dt):
        self.J = J
        self.J_inv = np.linalg.inv(J)
        self.omega = omega
        self.dt = dt

    def step(self, angular_momentum: np.ndarray, angular_torque: np.ndarray, external_torque: np.ndarray):
        matrix_mult = np.dot(self.J, self.omega)
        cross1 = np.cross(self.omega, matrix_mult)
        cross2 = np.cross(self.omega, angular_momentum)
        addition = external_torque - cross1 - cross2 - angular_torque

        domega_dt = np.dot(self.J_inv, addition)
        # Integration
        self.omega += self.dt * domega_dt

    def step_linear(self, angular_momentum: np.ndarray, angular_torque: np.ndarray, external_torque: np.ndarray):
        domega_dt = np.dot(self.J_inv, external_torque - angular_torque)
        self.omega += self.dt * domega_dt