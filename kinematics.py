import numpy as np

'''
    References:
    https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
'''


def quaternion_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quaternion_inverse(q):
    return (1 / (np.power(np.linalg.norm(q), 2))) * quaternion_conjugate(q)


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions q1 and q2.

    Parameters:
        q1 (numpy.array): Quaternion represented as a numpy array [x, y, z, w].
        q2 (numpy.array): Quaternion represented as a numpy array [x, y, z, w].

    Returns:
        numpy.array: Resultant quaternion after multiplication.
    """
    # Extract components
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    # Calculate the multiplication
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return np.array([x, y, z, w],dtype=np.float64)
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
