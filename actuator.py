import numpy as np


class ReactionWheel:
    def __init__(self, Jm, Bm, Ke, Kt, R, L, dt=1e-3):
        self.Jm = np.array(3 * [Jm])
        self.Bm = np.array(3 * [Bm])
        self.Ke = np.array(3 * [Ke])
        self.Kt = np.array(3 * [Kt])
        self.R = np.array(3 * [R])
        self.L = np.array(3 * [L])
        self.dt = dt

        self.domegadt = np.array([0.0, 0.0, 0.0])
        self.current = np.array([0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0, 0.0])
        self.torque = np.array([0.0, 0.0, 0.0])
        self.momentum = np.array([0.0, 0.0, 0.0])
        self.angles = np.array([0.0, 0.0, 0.0])

    def update(self, Va):
        # Torque dynamics
        dCurrentdt = np.array([0.0, 0.0, 0.0])
        dOmegadt = np.array([0.0, 0.0, 0.0])

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
    def __init__(self, num=100, area=1):
        self.num = num
        self.area = area
        self.torque = np.array([0.0, 0.0, 0.0])

    def update(self, current, B):
        prod1: np.ndarray = (self.num * self.area) * current
        self.torque = np.cross(prod1, B)


class ReactionWheel2:
    def __init__(self, Jm=0.01, Jr=3e-6, Bm=0.1, Kb=0.01,
                 Ki=0.01, R=1, L=0.5,
                 dt=1e-3):
        self.Jm = np.array(3 * [Jm])
        self.Jr = np.array(3 * [Jr])
        self.Bm = np.array(3 * [Bm])
        self.Ki = np.array(3 * [Ki])
        self.Kb = np.array(3 * [Kb])
        self.R = np.array(3 * [R])
        self.L = np.array(3 * [L])
        self.dt = dt

        self.domegadt = np.array([0.0, 0.0, 0.0])
        self.current = np.array([0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0, 0.0])
        self.torque = np.array([0.0, 0.0, 0.0])
        self.momentum = np.array([0.0, 0.0, 0.0])
        self.angles = np.array([0.0, 0.0, 0.0])

    def update(self, Va):
        # Torque dynamics
        dCurrentdt = np.array([0.0, 0.0, 0.0])
        dOmegadt = np.array([0.0, 0.0, 0.0])

        for i in range(0, 3):
            dCurrentdt[i] = (Va[i] / self.L[i]) - (self.R[i] / self.L[i]) * self.current[i] - (
                    self.Kb[i] * self.omega[i] / self.L[i])
            dOmegadt[i] = (self.Ki[i] / self.Jm[i]) * self.current[i] - (self.Bm[i] / self.Jm[i]) * self.omega[i]

        self.domegadt = dOmegadt
        self.torque = self.Jr * self.domegadt
        self.omega += self.dt * self.domegadt
        self.momentum = self.Jr * self.omega
        self.current += self.dt * dCurrentdt
        self.angles += self.dt * self.omega
