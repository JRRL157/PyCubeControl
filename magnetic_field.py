from math import sqrt, sin, cos

import numpy as np


class EarthMagneticFieldSimplified:

    def __init__(self, t0: np.float64 = 0, dt: np.float64 = 1e-3):
        self.b = np.array([0, 0, 0], dtype=np.float64)
        self.dt = dt
        self.t = t0

    def update_inertial_frame(self):
        self.b = (1 / (2 * (self.t + 1))) * np.array(
            [self.t + 2, sqrt(3 * self.t * self.t + 4 * self.t) * sin(self.t),
             sqrt(3 * self.t * self.t + 4 * self.t) * cos(self.t)])
        print("B Inertial Frame: ", self.b)
        self.t += self.dt
