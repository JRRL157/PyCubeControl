import matplotlib.pyplot as plt
import numpy as np

from actuator import ReactionWheel, Magnetorquer
from comparator import Comparator
from controller import PID, BDot
from dynamics import Dynamics
from kinematics import Kinematics
from magnetic_field import EarthMagneticFieldSimplified

try:
    import sim
except:
    print('"sim.py" could not be imported.')

# Constants
dt = 0.0001
J = np.array([[0.002169666666667, 0, 0],
              [0, 0.002169666666667, 0],
              [0, 0, 0.002169666666667]])

# Classes declaration
pid = PID(kp=1.0, ki=1.0, kd=0.05, dt=dt)
bdot = BDot(kp=1, num=100, area=1, MAX_CURRENT=1)
dynamics = Dynamics(J, omega=np.array([25, 5, 1]), dt=dt)
comparator = Comparator(input1_sgn=True, input2_sgn=False)
reaction_wheel = ReactionWheel(Kt=0.1, Jm=0.01, L=0.5, R=1, Ke=0.1, dt=dt)
magnetorquer = Magnetorquer(num=100, area=1)
kinematics = Kinematics(dt, np.array([0, 0, 0, 1]))
earth_magnetic_field = EarthMagneticFieldSimplified(t0=0, dt=dt)

# Reference
SP = np.array([0.0, 0.0, 0.0])

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
    # Updating Earth Magnetic Field from Inertial Frame
    earth_magnetic_field.update_inertial_frame()

    # Feedback error
    comparator.compare(SP, dynamics.omega)

    # Controllers
    pid.step(comparator.output)
    bdot.update(comparator.output, kinematics.b_body)

    # Actuators
    reaction_wheel.update(pid.voltage)
    magnetorquer.update(bdot.current, kinematics.b_body)

    # Spacecraft Dynamics
    dynamics.step(angular_momentum=reaction_wheel.momentum, angular_torque=reaction_wheel.torque,
                  external_torque=magnetorquer.torque)

    # Kinematics
    kinematics.update_quaternion(dynamics.omega)
    kinematics.update_magnetic_field_body_frame(earth_magnetic_field.b)

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

    #if np.abs(error[-1][0]) <= 1e-3:
    if n >= 50000:
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
