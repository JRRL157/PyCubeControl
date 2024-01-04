import numpy as np


class DCMotor:
    def __init__(self, resistance: np.float64, inductance: np.float64, kv: np.float64, torque_constant: np.float64,
                 inertia: np.float64, dt: np.float64):
        self.resistance = resistance
        self.inductance = inductance
        self.inertia = inertia # Motor inertia (kg.m^2)
        self.kv = kv  # Motor velocity constant (rad/s/V)
        self.torque_constant = torque_constant  # Motor torque constant (Nm/A)
        self.dt = dt

        self.current = 0.0
        self.angular_velocity = 0.0
        self.torque = 0.0

    def apply_voltage(self, voltage):
        # Motor dynamics (simplified model)
        back_emf = self.kv * self.angular_velocity
        voltage_net = voltage - back_emf

        # Calculate current using simplified electrical model (Ohm's Law and Kirchhoff's Voltage Law)
        current_change = (voltage_net - self.current * self.resistance) / self.inductance * self.dt
        self.current += current_change

        # Calculate torque based on current and motor torque constant
        self.torque = self.current * self.torque_constant

        # Angular acceleration (simplified model)
        angular_acceleration = self.torque / self.inertia
        self.angular_velocity += angular_acceleration * self.dt

class ReactionWheel:
    def __init__(self, inertia: np.float64, friction: np.float64, dt: np.float64,
                 max_angular_velocity: np.float64 = None):
        self.inertia = inertia
        self.friction = friction
        self.dt = dt
        self.max_angular_velocity = max_angular_velocity

        self.angular_velocity = np.float64(0.0)
        self.angular_acceleration = np.float64(0.0)

    def update(self, torque):
        # Torque dynamics
        self.angular_acceleration = torque / self.inertia - self.friction * self.angular_velocity / self.inertia
        self.angular_velocity += self.angular_acceleration * self.dt

        if self.max_angular_velocity is not None:
            self.angular_velocity = max(min(self.angular_velocity, self.max_angular_velocity),
                                        -self.max_angular_velocity)

        return self.angular_velocity
