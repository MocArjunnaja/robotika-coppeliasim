import sim
import sys
import time
import keyboard
import numpy as np

NO_DETECTION = 1.0  # distance value when sensor detects nothing (m)
D_REF = 0.4         # desired distance to object (m)
THETA_REF = 0.      # desired orientation: object right in front (S3 = S6)
K1 = 0.5            # distance gain
K2 = 0.5            # orientation gain
VELO_NORM = 1.5     # max wheel angular velocity after normalization (rad/s)

def connectSimulator():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1: print('Connected to remote API server.')
    else:
        print('Connection unsuccesful, program ended.')
        sys.exit()
    return clientID

def getSensorHandle(clientID):
    isNewCoppeliasim = True
    sensorsHandle = np.array([])
    for i in range(16):
        if(isNewCoppeliasim):
           sensorHandle = sim.simxGetObjectHandle(
                clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)[1]
        else:
            sensorHandle = sim.simxGetObjectHandle(
                clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
        _, _, _, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle,sensorHandle)
    sensorsHandle = np.int32(sensorsHandle)
    return sensorsHandle

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

def getDistance(clientID, sensorsHandle):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensorsHandle[i], sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = NO_DETECTION
        distances = np.append(distances,distance)
    return distances

def setRobotMotion(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)

def setVelocityUsingKeyboard(keyFwd, keyRev, veloID, veloStep, veloMax, veloRef):
    if keyboard.is_pressed(keyFwd):
        veloRef[veloID] += veloStep
        if veloRef[veloID] > veloMax:
            veloRef[veloID] = veloMax
    elif keyboard.is_pressed(keyRev):
        veloRef[veloID] -= veloStep
        if veloRef[veloID] < -veloMax:
            veloRef[veloID] = -veloMax
    else:
        # No key pressed: velocity decays back to zero
        if veloRef[veloID] > 0:
            veloRef[veloID] -= veloStep
            if veloRef[veloID] < 0.:
                veloRef[veloID] = 0
        else:
            veloRef[veloID] += veloStep
            if veloRef[veloID] > 0:
                veloRef[veloID] = 0

def setRobotMotionUsingkeyboard(veloRef):
    setVelocityUsingKeyboard('w', 's',0,0.025,0.5,veloRef)
    setVelocityUsingKeyboard('a', 'd',1,0.025,1.5,veloRef)

def frontSensorsSelection(distances):
    # S1..S16 on the slide = sensor[0..15], so S3..S6 = sensor[2..5]
    return distances[2], distances[3], distances[4], distances[5]

def estimateObject(s3, s4, s5, s6):
    d = min(s4, s5)
    theta = s6 - s3
    return d, theta

def objectFollowerControl(d, theta):
    # Slide: v = K1(d_ref - d), w = K2(theta_ref - theta). Both signs are flipped
    # here so K1, K2 stay positive: forward when object is far, and turn toward
    # the object (object on the left -> S3 < S6 -> theta > 0 -> w > 0, CCW)
    v = K1*(d - D_REF)
    w = K2*(theta - THETA_REF)
    return [v, w]

def initInvKinematics():
    wheelradius = 0.195/2
    halfwidth = 0.381/2
    R2 = wheelradius/2
    RL = wheelradius/(2*halfwidth)
    kinemmat = np.matrix([[R2,R2],[RL,-RL]])
    invkinemmat = kinemmat.I
    return invkinemmat

def solveKinematics(invkinemat, velo):
    # Returns [phi_R, phi_L]
    velomat = np.matrix([[velo[0]],[velo[1]]])
    veloangmat = invkinemat*velomat
    return [veloangmat[0,0],veloangmat[1,0]]

def velocityNormalization(veloAng, veloNorm):
    vr = veloAng[0]
    vl = veloAng[1]
    velo_max = max(abs(vr), abs(vl))
    if velo_max > veloNorm:
        vr = veloNorm/velo_max*vr
        vl = veloNorm/velo_max*vl
    return [vr, vl]


print('Program started')
print('Keys: [F] object follower, [M] manual (W/A/S/D), [Esc] quit')
client_id = connectSimulator()
motors_handle = getMotorHandle(client_id)
sensors_handle = getSensorHandle(client_id)
inv_kinem_const = initInvKinematics()
samp_time = 0.1
n = 1
velo_ang_zero = [0,0]
velo_cmd = [0.,0.]
mode = 'follower'
time_start = time.time()
while (True):
    t_now = time.time()-time_start
    if t_now >= samp_time*n:
        n += 1.
        object_distances=getDistance(client_id,sensors_handle)
        # Mode selection
        if keyboard.is_pressed('f') and mode != 'follower':
            mode = 'follower'
            velo_cmd = [0.,0.]
        elif keyboard.is_pressed('m') and mode != 'manual':
            mode = 'manual'
            velo_cmd = [0.,0.]
        # Motion command
        s3, s4, s5, s6 = frontSensorsSelection(object_distances)
        obj_d, obj_theta = estimateObject(s3, s4, s5, s6)
        if mode == 'follower':
            if min(s3, s4, s5, s6) >= NO_DETECTION:
                # No object in front: stop instead of driving blind
                velo_cmd = [0.,0.]
            else:
                velo_cmd = objectFollowerControl(obj_d, obj_theta)
        else:
            setRobotMotionUsingkeyboard(velo_cmd)
        # Inverse kinematics -> [phi_R, phi_L]
        velo_ang_cmd = solveKinematics(inv_kinem_const, velo_cmd)
        # Velocity normalization -> [phi_RN, phi_LN]
        velo_ang_norm = velocityNormalization(velo_ang_cmd, VELO_NORM)
        # Robot motion simulation (motors_handle order is left, right)
        setRobotMotion(client_id,motors_handle,[velo_ang_norm[1],velo_ang_norm[0]])
        print('t = ', np.round(t_now, 1), ' mode = ', mode,
              ' velocity command (v, w) = ', np.round(velo_cmd, 3))
        print('  S3..S6 = ', np.round([s3, s4, s5, s6], 3),
              ' d = ', np.round(obj_d, 3), ' theta = ', np.round(obj_theta, 3))
        print('  wheel velocity (phi_RN, phi_LN) = ', np.round(velo_ang_norm, 3))
    if keyboard.is_pressed('esc'):
        setRobotMotion(client_id,motors_handle,velo_ang_zero)
        break
#Simulation Finished
sim.simxFinish(client_id)
print('program ended')
