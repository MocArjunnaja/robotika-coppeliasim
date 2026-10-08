import sim
import sys
import math
import time
import keyboard
import numpy as np

# Waypoints visited in this order (object alias in CoppeliaSim scene)
DISC_NAMES = ['/Disc[0]', '/Disc[1]', '/Disc[2]', '/Disc[3]', '/Disc[4]', '/Disc[5]']
K1 = 0.5            # gain e_x
K2 = 0.5            # gain e_y
K3 = 0.5            # gain e_gamma (final orientation)
E_TOL = 0.05        # position tolerance (m)
GAMMA_TOL = 0.05    # orientation tolerance (rad)
VELO_NORM = 3.0     # max wheel angular velocity after normalization (rad/s)
WHEEL_RADIUS = 0.195/2
HALF_WIDTH = 0.381/2

def connectSimulator():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1: print('Connected to remote API server.')
    else:
        print('Connection unsuccesful, program ended.')
        sys.exit()
    return clientID

def getMotorHandle(clientID):
    isNewCoppeliaSim = True
    if (isNewCoppeliaSim):
        motorRightHandle = sim.simxGetObjectHandle(
            clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)[1]
        motorLeftHandle = sim.simxGetObjectHandle(
            clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)[1]
    else:
        motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)[1]
        motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1]
    motorsHandle = (motorLeftHandle, motorRightHandle)
    return motorsHandle

def getRobotHandle(clientID):
    isNewCoppeliaSim = True
    if (isNewCoppeliaSim):
        robotHandle = sim.simxGetObjectHandle(
            clientID, '/PioneerP3DX', sim.simx_opmode_blocking)[1]
    else:
        robotHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)[1]
    return robotHandle

def getDiscHandles(clientID):
    discsHandle = []
    for name in DISC_NAMES:
        error, discHandle = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
        if error != sim.simx_return_ok:
            print('Object', name, 'not found in scene, program ended.')
            sim.simxFinish(clientID)
            sys.exit()
        discsHandle.append(discHandle)
    return discsHandle

def startPoseStreaming(clientID, handle):
    _ = sim.simxGetObjectPosition(clientID, handle, -1, sim.simx_opmode_streaming)
    _ = sim.simxGetObjectOrientation(clientID, handle, -1, sim.simx_opmode_streaming)

def getPose(clientID, handle):
    # Robot localization: position (x, y) and yaw (gamma) relative to world
    error_p, position = sim.simxGetObjectPosition(clientID, handle, -1, sim.simx_opmode_buffer)
    error_o, orientation = sim.simxGetObjectOrientation(clientID, handle, -1, sim.simx_opmode_buffer)
    valid = (error_p == sim.simx_return_ok) and (error_o == sim.simx_return_ok)
    return valid, position[0], position[1], orientation[2]

def setRobotMotion(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)

def wrapAngle(angle):
    # Keep angle in [-pi, pi] so the robot always turns the short way
    return math.atan2(math.sin(angle), math.cos(angle))

def poseControl(e_x, e_y, e_gamma, gamma_act):
    theta = math.atan2(e_y, e_x)
    if (abs(e_x) <= E_TOL) and (abs(e_y) <= E_TOL):
        # Position reached: only correct final orientation
        x_dotc = 0.
        y_dotc = 0.
        gamma_dotc = K3*e_gamma
    else:
        x_dotc = K1*e_x
        y_dotc = K2*e_y
        gamma_dotc = wrapAngle(theta - gamma_act)
    return theta, x_dotc, y_dotc, gamma_dotc

def initInvKinematics(theta):
    R2 = WHEEL_RADIUS/2
    RL = WHEEL_RADIUS/(2*HALF_WIDTH)
    kinemmat = np.array([[R2*math.cos(theta), R2*math.cos(theta)],
                         [R2*math.sin(theta), R2*math.sin(theta)],
                         [RL, -RL]])
    invkinemmat = np.linalg.pinv(kinemmat)
    return invkinemmat

def solveKinematics(invkinemat, x_dotc, y_dotc, gamma_dotc):
    # Wheel velocity generator, returns [phi_R, phi_L]
    velomat = np.array([x_dotc, y_dotc, gamma_dotc])
    veloangmat = invkinemat @ velomat
    return [veloangmat[0], veloangmat[1]]

def velocityNormalization(veloAng, veloNorm):
    vr = veloAng[0]
    vl = veloAng[1]
    velo_max = max(abs(vr), abs(vl))
    if velo_max > veloNorm:
        vr = veloNorm/velo_max*vr
        vl = veloNorm/velo_max*vl
    return [vr, vl]


print('Program started')
print('Waypoints:', DISC_NAMES, ' [Esc] quit')
client_id = connectSimulator()
motors_handle = getMotorHandle(client_id)
robot_handle = getRobotHandle(client_id)
discs_handle = getDiscHandles(client_id)
startPoseStreaming(client_id, robot_handle)
for disc_handle in discs_handle:
    startPoseStreaming(client_id, disc_handle)
samp_time = 0.1
n = 1
velo_ang_zero = [0,0]
target = 0
time_start = time.time()
while (True):
    t_now = time.time()-time_start
    if t_now >= samp_time*n:
        n += 1.
        if target >= len(DISC_NAMES):
            setRobotMotion(client_id,motors_handle,velo_ang_zero)
        else:
            # Robot localization
            valid_r, x_act, y_act, gamma_act = getPose(client_id, robot_handle)
            valid_d, x_ref, y_ref, gamma_ref = getPose(client_id, discs_handle[target])
            if valid_r and valid_d:
                # Pose control
                e_x = x_ref - x_act
                e_y = y_ref - y_act
                e_gamma = wrapAngle(gamma_ref - gamma_act)
                theta, x_dotc, y_dotc, gamma_dotc = poseControl(e_x, e_y, e_gamma, gamma_act)
                # Wheel velocity generator -> [phi_R, phi_L]
                inv_kinem_const = initInvKinematics(theta)
                velo_ang_cmd = solveKinematics(inv_kinem_const, x_dotc, y_dotc, gamma_dotc)
                # Velocity normalization -> [phi_RN, phi_LN]
                velo_ang_norm = velocityNormalization(velo_ang_cmd, VELO_NORM)
                # Robot motion simulation (motors_handle order is left, right)
                setRobotMotion(client_id,motors_handle,[velo_ang_norm[1],velo_ang_norm[0]])
                print('t = ', np.round(t_now, 1), ' target = ', DISC_NAMES[target],
                      ' (e_x, e_y, e_gamma) = ', np.round([e_x, e_y, e_gamma], 3))
                print('  wheel velocity (phi_RN, phi_LN) = ', np.round(velo_ang_norm, 3))
                # Waypoint reached: position and orientation inside tolerance
                if (abs(e_x) <= E_TOL) and (abs(e_y) <= E_TOL) and (abs(e_gamma) <= GAMMA_TOL):
                    print('Reached', DISC_NAMES[target])
                    target += 1
                    if target >= len(DISC_NAMES):
                        setRobotMotion(client_id,motors_handle,velo_ang_zero)
                        print('All waypoints reached. Press [Esc] to quit.')
    if keyboard.is_pressed('esc'):
        setRobotMotion(client_id,motors_handle,velo_ang_zero)
        break
#Simulation Finished
sim.simxFinish(client_id)
print('program ended')
