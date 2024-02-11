import numpy as np

'''
    References:

    https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
'''


class Kinematics:

    def __init__(self, dt: np.float64, q: np.ndarray = np.array([0, 0, 0, 1])):
        self.dt = dt
        self.q = q
        self.dqdt = np.array([0, 0, 0, 1], dtype=np.float64)

    def update_quaternion(self, w):
        skew_matrix_omega = np.array([[0, -w[0], -w[1], -w[2]],
                                      [w[0], 0, w[2], -w[1]],
                                      [w[1], -w[2], 0, w[0]],
                                      [w[2], w[1], -w[0], 0]])
        self.dqdt = 0.5 * np.dot(skew_matrix_omega, self.q)
        self.q += self.dqdt * self.dt
