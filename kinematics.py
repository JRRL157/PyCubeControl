import numpy as np

'''
    References:
    https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
'''


def quaternion_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quaternion_inverse(q):
    return (1 / (np.power(np.linalg.norm(q), 2))) * quaternion_conjugate(q)


def ref_frameA_to_ref_frameB(v, q):
    q_inv = quaternion_conjugate(q)

    # Reshape
    _v = v.reshape(v.shape[0], -1)
    _q = q.reshape(1, q.shape[0])
    _q_inv = q_inv.reshape(1, q_inv.shape[0])

    mult1 = np.dot(_v, _q_inv)
    mult2 = np.dot(_q, mult1.T)

    return mult2.T.reshape(3, )


class Kinematics:

    def __init__(self, dt: np.float64, q: np.ndarray = np.array([0, 0, 0, 1])):
        self.dt = dt
        self.q = q
        self.dqdt = np.array([0, 0, 0, 1], dtype=np.float64)
        self.b_body = np.array([0.01, 0.01, 0.01], dtype=np.float64)

    def update_quaternion(self, w):
        skew_matrix_omega = np.array([[0, -w[0], -w[1], -w[2]],
                                      [w[0], 0, w[2], -w[1]],
                                      [w[1], -w[2], 0, w[0]],
                                      [w[2], w[1], -w[0], 0]])
        self.dqdt = 0.5 * np.dot(skew_matrix_omega, self.q)
        self.q += self.dqdt * self.dt
        self.q = (1 / np.linalg.norm(self.q)) * self.q

    def update_magnetic_field_body_frame(self, b):
        self.b_body = ref_frameA_to_ref_frameB(b, self.q)
