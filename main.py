import numpy as np

from actuator import ReactionWheel,DCMotor
from comparator import Comparator
from controller import PID
from dynamics import Dynamics

# Constants
dt = np.float64(1e3)
Jw = 2.887e-03
Jb = 2.042e-02
Jr = np.float64(Jw / Jb)

# Classes declaration
pid = PID(kp=np.float64(1.0), ki=np.float64(0.1), kd=np.float64(0.01), dt=dt)
dynamics = Dynamics(jr=Jr, omega=np.array([3.0, 0.0, 0.0], dtype=np.float64), dt=dt)
comparator = Comparator(input1_sgn=True, input2_sgn=False)
dc_motor = DCMotor(resistance=1.0, inductance=0.01, kv=0.1, torque_constant=0.01, inertia=0.00002, dt=dt)
reaction_wheel = ReactionWheel(inertia=np.float64(0.01), friction=np.float64(0.001), dt=dt)

# Reference
SP = np.array([0.0, 0.0, 0.0], dtype=np.float64)


n = 0
error = []
torque = []
cube_omega = []

while True:
    t = n * dt
    comparator.compare(SP, dynamics.omega)
    pid.step(comparator.output)
    dc_motor.apply_voltage(pid.get_output())
    reaction_wheel.update(dc_motor.torque)
    dynamics.step(input_velocity=reaction_wheel.angular_velocity,
                  input_acceleration=reaction_wheel.angular_acceleration)

    error.append(comparator.output)
    torque.append(dc_motor.torque)
    cube_omega.append(dynamics.omega)

# Plot results
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(time_points, angular_positions, label='Current error of the cubesat angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_points, angular_velocities, label='Motors torque')
plt.xlabel('Time (s)')
plt.ylabel('Torque')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_points, control_signals, label='Cubesat Angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Omega')
plt.legend()

plt.tight_layout()
plt.show()