import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel
from comparator import Comparator
from controller import PID
from dynamics import Dynamics

# Constants
dt = np.float64(0.0001)
J = np.array([[0.002169666666667, 0, 0], [0, 0.002169666666667, 0], [0, 0, 0.002169666666667]])

# Classes declaration
pid = PID(kp=np.float64(1.0), ki=np.float64(1.0), kd=np.float64(0.0), dt=dt)
dynamics = Dynamics(J, omega=np.array([3, 1, 2], dtype=np.float64), dt=dt)
comparator = Comparator(input1_sgn=True, input2_sgn=False)
reaction_wheel = ReactionWheel(dt=dt)

# Reference
SP = np.array([0.0, 0.0, 0.0], dtype=np.float64)

n = 0
m = 1000
error = []
rwOmega = []
cube_omega = []
time_points = []
pidOutput = []

while True:
    comparator.compare(SP, dynamics.omega)
    pid.step(comparator.output)

    '''    
    try:
        if np.abs(error[-1][0]) <= 1e-2:
            reaction_wheel.update(np.array([0, 0, 0]))
            print("N = ", n, " w = ", reaction_wheel.omega)
        else:
            reaction_wheel.update(pid.output)
    except Exception as e:
        print(e.__traceback__)
    '''
    reaction_wheel.update(pid.output)

    dynamics.step(angular_momentum=-reaction_wheel.momentum,
                  angular_torque=-reaction_wheel.torque,
                  external_torque=np.array([0, 0, 0]))

    if n <= 10:
        time_points.append(n * dt)
        error.append(comparator.output)
        pidOutput.append(pid.output)
        rwOmega.append(reaction_wheel.omega)
        cube_omega.append(dynamics.omega)

    try:
        if n % 1000 == 0:
            time_points.append(n * dt)
            error.append(comparator.output)
            pidOutput.append(pid.output)
            rwOmega.append(reaction_wheel.omega)
            cube_omega.append(dynamics.omega)
            print("N = ", n, ", error = ", error[-1])
    except:
        None

    if np.abs(error[-1][0]) <= 1e-3 or n == 1000000:
        break

    n += 1

# Plot results
plt.figure(figsize=(10, 6))

plt.subplot(4, 1, 1)
plt.plot(time_points, error, label='Current error of the cubesat angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time_points, rwOmega, label='RW Omega(rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Omega(rad/s)')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time_points, cube_omega, label='Cubesat Angular velocity')
plt.xlabel('Time (s)')
plt.ylabel('Omega')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_points, pidOutput, label='Saída de Controle')
plt.xlabel('Time (s)')
plt.ylabel('controle')
plt.legend()

plt.tight_layout()
plt.show()
