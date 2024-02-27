import numpy as np


class ReactionWheel:
    def __init__(self, Jm: np.float64 = 0.01, Bm: np.float64 = 0.1, Ke: np.float64 = 0.01,
                 Kt: np.float64 = 0.01, R: np.float64 = 1, L: np.float64 = 0.5,
                 dt: np.float64 = 1e-3):
        self.Jm = np.array(3 * [Jm], dtype=np.float64)
        self.Bm = np.array(3 * [Bm], dtype=np.float64)
        self.Ke = np.array(3 * [Ke], dtype=np.float64)
        self.Kt = np.array(3 * [Kt], dtype=np.float64)
        self.R = np.array(3 * [R], dtype=np.float64)
        self.L = np.array(3 * [L], dtype=np.float64)
        self.dt = dt

        self.domegadt = np.array([0, 0, 0], dtype=np.float64)
        self.current = np.array([0, 0, 0], dtype=np.float64)
        self.omega = np.array([0, 0, 0], dtype=np.float64)
        self.torque = np.array([0, 0, 0], dtype=np.float64)
        self.momentum = np.array([0, 0, 0], dtype=np.float64)
        self.angles = np.array([0, 0, 0], dtype=np.float64)

    def update(self, Va):
        # Torque dynamics
        dCurrentdt = np.array([0, 0, 0], dtype=np.float64)
        dOmegadt = np.array([0, 0, 0], dtype=np.float64)

        for i in range(0, 3):
            dCurrentdt[i] = (Va[i] / self.L[i]) - (self.R[i] / self.L[i]) * self.current[i] - (
                    self.Ke[i] * self.omega[i] / self.L[i])
            dOmegadt[i] = (self.Kt[i] / self.Jm[i]) * self.current[i] - (self.Bm[i] / self.Jm[i]) * self.omega[i]

        self.domegadt = dOmegadt
        self.torque = self.Jm * self.domegadt
        self.omega += self.dt * self.domegadt
        self.momentum = self.Jm * self.omega
        self.current += self.dt * dCurrentdt
        self.angles += self.dt * self.omega


class Magnetorquer:
    def __init__(self, num: np.float64 = 100, area: np.float64 = 1):
        self.num = num
        self.area = area
        self.torque = np.array([0, 0, 0], dtype=np.float64)

    def update(self, current, B):
        prod1: np.ndarray = (self.num * self.area) * current
        self.torque = np.cross(prod1, B)


class ReactionWheel2:
    def __init__(self, Jm: np.float64 = 0.01, Jr: np.float64 = 3e-6, Bm: np.float64 = 0.1, Kb: np.float64 = 0.01,
                 Ki: np.float64 = 0.01, R: np.float64 = 1, L: np.float64 = 0.5,
                 dt: np.float64 = 1e-3):
        self.Jm = np.array(3 * [Jm], dtype=np.float64)
        self.Jr = np.array(3 * [Jr], dtype=np.float64)
        self.Bm = np.array(3 * [Bm], dtype=np.float64)
        self.Ki = np.array(3 * [Ki], dtype=np.float64)
        self.Kb = np.array(3 * [Kb], dtype=np.float64)
        self.R = np.array(3 * [R], dtype=np.float64)
        self.L = np.array(3 * [L], dtype=np.float64)
        self.dt = dt

        self.domegadt = np.array([0, 0, 0], dtype=np.float64)
        self.current = np.array([0, 0, 0], dtype=np.float64)
        self.omega = np.array([0, 0, 0], dtype=np.float64)
        self.torque = np.array([0, 0, 0], dtype=np.float64)
        self.momentum = np.array([0, 0, 0], dtype=np.float64)
        self.angles = np.array([0, 0, 0], dtype=np.float64)

    def update(self, Va):
        # Torque dynamics
        dCurrentdt = np.array([0, 0, 0], dtype=np.float64)
        dOmegadt = np.array([0, 0, 0], dtype=np.float64)

        for i in range(0, 3):
            dCurrentdt[i] = (Va[i] / self.L[i]) - (self.R[i] / self.L[i]) * self.current[i] - (
                    self.Kb[i] * self.omega[i] / self.L[i])
            dOmegadt[i] = (self.Ki[i] / self.Jm[i]) * self.current[i] - (self.Bm[i] / self.Jm[i]) * self.omega[i]

        self.domegadt = dOmegadt
        self.torque = -self.Jr * self.domegadt
        self.omega += self.dt * self.domegadt
        self.momentum = -self.Jr * self.omega
        self.current += self.dt * dCurrentdt
        self.angles += self.dt * self.omega