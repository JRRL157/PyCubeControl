import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel
from controller import LQR_Attitude
from dynamics import Dynamics
from kinematics import Kinematics

try:
    import sim
except:
    print('"sim.py" could not be imported.')

# Constants
dt = 10e-3
J = np.array([[0.002169666666667, 0, 0],
              [0, 0.002169666666667, 0],
              [0, 0, 0.002169666666667]])

# Classes declaration
lqr_attitude = LQR_Attitude(K=np.array([0.4739, -49.5965, -69.7886]))
dynamics = Dynamics(J, omega=np.array([0.0,0.0,0.0], dt=dt)
reaction_wheel = ReactionWheel(Kt=0.0264, Jm=0.0015, L=0.48, R=7, Ke=0.0088, dt=dt)
kinematics = Kinematics(dt, np.array([0, 0, 0, 1]))

# Reference
quat_ref = np.array([0.634, 0.0151, 0.654, 0.413])

n = 0
m = 1000
error = []
rwOmega = []
cube_omega = []
time_points = []
lqrVoltage = []

sim.simxFinish(-1)  # close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 10000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')

ret, motor_x = sim.simxGetObjectHandle(clientID, "motor_x", sim.simx_opmode_oneshot_wait)
ret, motor_y = sim.simxGetObjectHandle(clientID, "motor_y", sim.simx_opmode_oneshot_wait)
ret, motor_z = sim.simxGetObjectHandle(clientID, "motor_z", sim.simx_opmode_oneshot_wait)
ret, cubesat = sim.simxGetObjectHandle(clientID, "cubesat", sim.simx_opmode_oneshot_wait)

while clientID != -1:

    # Controllers
    lqr_attitude.update(quat_ref, np.concatenate([reaction_wheel.current,
                                                  reaction_wheel.omega,
                                                  kinematics.q], axis=0))

    print("Voltage = ", lqr_attitude.u)
    # Actuators
    reaction_wheel.update(lqr_attitude.u)

    # Spacecraft Dynamics
    dynamics.step(angular_momentum=reaction_wheel.momentum, angular_torque=reaction_wheel.torque,
                  external_torque=np.array([0.0,0.0,0.0])

    # Kinematics
    kinematics.update_quaternion(dynamics.omega)

    # CoppeliaSim Objects commands
    sim.simxSetJointTargetPosition(clientID, motor_x, reaction_wheel.angles[0], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, motor_y, reaction_wheel.angles[1], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, motor_z, reaction_wheel.angles[2], sim.simx_opmode_streaming)
    sim.simxSetObjectQuaternion(clientID, cubesat, quaternion=kinematics.q.tolist(),
                                relativeToObjectHandle=-1,
                                operationMode=sim.simx_opmode_streaming)

    if n <= 10:
        time_points.append(n * dt)
        lqrVoltage.append(lqr_attitude.u)
        rwOmega.append(reaction_wheel.omega)
        cube_omega.append(dynamics.omega)

    try:
        if n % 1000 == 0:
            time_points.append(n * dt)
            lqrVoltage.append(lqr_attitude.u)
            rwOmega.append(reaction_wheel.omega)
            cube_omega.append(dynamics.omega)
    except:
        None

    # if np.abs(error[-1][0]) <= 1e-3:
    if n >= 50000:
        break

    n += 1

if clientID != -1:
    # Plot results
    plt.figure(figsize=(10, 6))

    plt.subplot(3, 1, 1)
    plt.plot(time_points, rwOmega, label='Motor Omega(rad/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Omega(rad/s)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_points, cube_omega, label='Cubesat Angular velocity(rad/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Omega(rad/s)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_points, lqrVoltage, label='Saída de Controle(Volt)')
    plt.xlabel('Time (s)')
    plt.ylabel('controle')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)
# Stop simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
