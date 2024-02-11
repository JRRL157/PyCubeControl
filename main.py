import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel
from comparator import Comparator
from controller import PID
from dynamics import Dynamics

try:
    import sim
except:
    print(
        '"sim.py" could not be imported. Check whether "sim.py" or the remoteApi library could not be found. Make sure both are in the same folder as this file')

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

sim.simxFinish(-1)  # close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 10000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')

ret, motor_x = sim.simxGetObjectHandle(clientID, "motor_x", sim.simx_opmode_oneshot_wait)
print(ret)
ret, motor_y = sim.simxGetObjectHandle(clientID, "motor_y", sim.simx_opmode_oneshot_wait)
print(ret)
ret, motor_z = sim.simxGetObjectHandle(clientID, "motor_z", sim.simx_opmode_oneshot_wait)
print(ret)
ret, cubesat = sim.simxGetObjectHandle(clientID,"cubesat",sim.simx_opmode_oneshot_wait)
print(ret)

while clientID != -1:
    comparator.compare(SP, dynamics.omega)
    pid.step(comparator.output)
    reaction_wheel.update(pid.output)
    sim.simxSetJointTargetVelocity(clientID, motor_x, reaction_wheel.omega[0], sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, motor_y, reaction_wheel.omega[1], sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, motor_y, reaction_wheel.omega[2], sim.simx_opmode_streaming)

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

if clientID != -1:
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

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)
# Stop simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
