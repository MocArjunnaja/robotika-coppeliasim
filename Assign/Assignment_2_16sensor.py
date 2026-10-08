import sim
import sys
import time
import keyboard
import numpy as np

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
            distance = 2.0
        distances = np.append(distances,distance)
    return distances

def setRobotMotion(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)


print('Program started')
client_id = connectSimulator()
motors_handle = getMotorHandle(client_id)
sensors_handle = getSensorHandle(client_id)
samp_time = 0.1
n = 1
velo_ang_zero = [0,0]
velo_init=(0.0,0.0)
time_start = time.time()
while (True):
    t_now = time.time()-time_start
    if t_now >= samp_time*n:
        n += 1.
        object_distances=getDistance(client_id,sensors_handle)
        # Motion command using keyboard
        motors_velo=velo_init
        setRobotMotion(client_id,motors_handle,motors_velo)
        # Robot motion simulation
        print('t = ', np.round(t_now, 1))
        for i, distance in enumerate(object_distances):
            print('  sensor[', i, '] distance = ', np.round(distance, 3))
    if keyboard.is_pressed('esc'):
        setRobotMotion(client_id,motors_handle,velo_ang_zero)
        break
#Simulation Finished
sim.simxFinish(client_id)
print('program ended')
