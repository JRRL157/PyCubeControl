import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel
from comparator import Comparator
from controller import PID
from dynamics import Dynamics

# Constants
dt = np.float64(0.01)
Jw = 2.887e-03
Jb = 2.042e-02
Jr = np.float64(Jw / Jb)

# Classes declaration
pid = PID(kp=np.float64(10.0), ki=np.float64(1.0), kd=np.float64(0.05), dt=dt)
dynamics = Dynamics(jr=Jr, omega=np.array([2, 1, 0.8], dtype=np.float64), dt=dt)
comparator = Comparator(input1_sgn=True, input2_sgn=False)
reaction_wheel = ReactionWheel(dt=dt)

# Reference
SP = np.array([0.0, 0.0, 0.0], dtype=np.float64)

n = 0
error = []
rwOmega = []
cube_omega = []
time_points = []

while True:
    comparator.compare(SP, dynamics.omega)
    pid.step(comparator.output)
    reaction_wheel.update(pid.output)
    dynamics.step(input_velocity=reaction_wheel.omega,
                  input_acceleration=reaction_wheel.domegadt)

    n += 1

    time_points.append(n * dt)
    error.append(comparator.output)
    rwOmega.append(reaction_wheel.omega)
    cube_omega.append(dynamics.omega)

    if np.abs(error[-1][0]) <= 1e-3:
        break


# Plot results
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(time_points, error, label='Current error of the cubesat angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_points, rwOmega, label='RW Omega(rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Omega(rad/s)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_points, cube_omega, label='Cubesat Angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Omega')
plt.legend()

plt.tight_layout()
plt.show()
