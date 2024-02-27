import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel2
from comparator import Comparator
from controller import PID
from dynamics import Dynamics
from kinematics import Kinematics

try:
    import sim
except:
    print('"sim.py" could not be imported.')

# Constants
dt = np.float64(1e-2)
J = np.array([[0.002169666666667, 0, 0],
              [0, 0.002169666666667, 0],
              [0, 0, 0.002169666666667]])

# Classes declaration
pid = PID(kp=np.float64(84.774), ki=np.float64(0.0), kd=np.float64(18.7096218), dt=dt)
dynamics = Dynamics(J, omega=np.array([2, 5, 2], dtype=np.float64), dt=dt)
comparator = Comparator(input1_sgn=True, input2_sgn=False)
reaction_wheel = ReactionWheel2(Jm=0.0015, Jr=3e-6, Kb=9.22e-04, Ki=0.0264, Bm=0.0088, L=0.48, R=7, dt=dt)
kinematics = Kinematics(dt, np.array([0, 0, 0, 1], dtype=np.float64))

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
ret, motor_y = sim.simxGetObjectHandle(clientID, "motor_y", sim.simx_opmode_oneshot_wait)
ret, motor_z = sim.simxGetObjectHandle(clientID, "motor_z", sim.simx_opmode_oneshot_wait)
ret, cubesat = sim.simxGetObjectHandle(clientID, "cubesat", sim.simx_opmode_oneshot_wait)

while clientID != -1:

    # Feedback error
    comparator.compare(SP, dynamics.omega)

    # Controllers
    pid.step(comparator.output)

    # Actuators
    reaction_wheel.update(pid.voltage)

    # Spacecraft Dynamics
    dynamics.step(angular_momentum=reaction_wheel.momentum, angular_torque=reaction_wheel.torque,
                  external_torque=np.array([0, 0, 0], dtype=np.float64))

    # Kinematics
    kinematics.update_quaternion(dynamics.omega)

    # CoppeliaSim Objects commands
    sim.simxSetJointTargetVelocity(clientID, motor_x, reaction_wheel.omega[0], sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, motor_y, reaction_wheel.omega[1], sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, motor_z, reaction_wheel.omega[2], sim.simx_opmode_streaming)
    sim.simxSetObjectQuaternion(clientID, cubesat, quaternion=kinematics.q.tolist(),
                                relativeToObjectHandle=-1,
                                operationMode=sim.simx_opmode_streaming)

    if n <= 10:
        time_points.append(n * dt)
        error.append(comparator.output)
        pidOutput.append(pid.voltage)
        rwOmega.append(reaction_wheel.omega)
        cube_omega.append(dynamics.omega)

    try:
        if n % 1000 == 0:
            time_points.append(n * dt)
            error.append(comparator.output)
            pidOutput.append(pid.voltage)
            rwOmega.append(reaction_wheel.omega)
            cube_omega.append(dynamics.omega)
            print("N = ", n, ", error = ", error[-1])
    except:
        None

    if np.abs(error[-1][0]) <= 1e-3: break
    # if n >= 50000:
    #    break

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
