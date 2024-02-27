from math import exp

import numpy as np

import kinematics


class PID:

    def __init__(self, kp: np.float64, kd: np.float64, ki: np.float64, N: np.float64 = 100, dt: np.float64 = 1e-3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.N = N
        self.t = 0

        self.prev_error = np.array([0, 0, 0], dtype=np.float64)
        self.integral = np.array([0, 0, 0], dtype=np.float64)
        self.voltage = np.array([0, 0, 0], dtype=np.float64)

    def step(self, error: np.ndarray):
        self.t += self.dt

        proportional = self.kp * error
        self.integral += self.ki * error * self.dt
        derivative = self.kd * (error - self.prev_error) / self.dt

        self.prev_error = error

        self.voltage = proportional + self.integral + derivative
        self.voltage = np.array(
            [-120 if x < -120 else (120 if x > 120 else x) for x in self.voltage])

    def get_output(self):
        return np.array([-12 if x <= -8192 else (12 if x >= 8192 else x / 8192.0) for x in self.voltage])


class BDot:
    def __init__(self, kp: np.float64 = 1.0, num: np.float64 = 100, area: np.float64 = 1, MAX_CURRENT: np.float64 = 1):
        self.kp = kp
        self.num = num
        self.area = area
        self.current = np.array([0, 0, 0], dtype=np.float64)
        self.MAX_CURRENT = MAX_CURRENT

    def update(self, error: np.ndarray, B):
        K = self.kp / np.power(np.linalg.norm(B), 2)
        cross_prod = np.cross(B, error)
        mult1 = K * cross_prod
        self.current = mult1 / (self.num * self.area)
        self.current = np.array([
            -self.MAX_CURRENT if x < -self.MAX_CURRENT else (self.MAX_CURRENT if x > self.MAX_CURRENT else x) for x in
            self.current])


class LQR_Stabilization:
    def __init__(self, K: np.ndarray = np.array([-0.0059, -1.4799, -0.4366])):
        self.K = K
        self.u = np.array([0, 0, 0], dtype=np.float64)

    def update(self, R_vector: np.ndarray, state_vector: np.ndarray):
        current_vector = state_vector[0:3]
        omega_motor_vector = state_vector[3:6]
        omega_vector = state_vector[6:9]

        error_vector = (R_vector - omega_vector)

        state_vector_first = np.array([current_vector[0], omega_motor_vector[0], error_vector[0]])
        state_vector_second = np.array([current_vector[1], omega_motor_vector[1], error_vector[1]])
        state_vector_third = np.array([current_vector[2], omega_motor_vector[2], error_vector[2]])

        u_first = np.dot(-self.K, state_vector_first)
        u_second = np.dot(-self.K, state_vector_second)
        u_third = np.dot(-self.K, state_vector_third)

        self.u = np.array([u_first, u_second, u_third])


class LQR_Attitude:
    def __init__(self, K: np.ndarray = np.array([0.4151, 0.3541, -3.1623])):
        self.K = K
        self.u = np.array([0, 0, 0], dtype=np.float64)

    def update(self, quat_ref: np.ndarray, state_vector: np.ndarray):
        print(state_vector)
        current_vector = state_vector[0:3]
        omega_motor_vector = state_vector[3:6]
        quat_obs = state_vector[6:10]

        quat_error = kinematics.quaternion_multiply(kinematics.quaternion_inverse(quat_obs), quat_ref)
        error_vector = -quat_error[1:4]

        u = np.array([0, 0, 0], dtype=np.float64)
        u[0] = -self.K[0] * current_vector[0] - self.K[1] * omega_motor_vector[0] + self.K[2] * error_vector[0]
        u[1] = -self.K[0] * current_vector[1] - self.K[1] * omega_motor_vector[1] + self.K[2] * error_vector[1]
        u[2] = -self.K[0] * current_vector[2] - self.K[1] * omega_motor_vector[2] + self.K[2] * error_vector[2]
        self.u = np.array([120 if x > 120 else -120 if x < -120 else x for x in u])
