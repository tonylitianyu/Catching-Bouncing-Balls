try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    vel=180
    accel=40
    jerk=80
    currentVel = 0
    currentAccel = 0
    maxVel = vel*math.pi/180
    maxAccel = accel*math.pi/180
    maxJerk = jerk*math.pi/180
    targetPos1 = 90*math.pi/180
    targetVel = 0

    # sensor
    res,posensor=vrep.simxGetObjectHandle(clientID,'LaserPointer_sensor',vrep.simx_opmode_blocking)
    code,state,point,obj,normv = vrep.simxReadProximitySensor(clientID,posensor,vrep.simx_opmode_streaming)
    print(state)
    print(point)
    print(obj)
    print(normv)


    # Load joint 1
    res,joint1=vrep.simxGetObjectHandle(clientID,'UR3_joint1',vrep.simx_opmode_blocking)

    code,pos = vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming)
    print(pos)
    vrep.simxSetJointForce(clientID,joint1,3,vrep.simx_opmode_oneshot) #set max force for this joint
    vrep.simxSetJointTargetPosition(clientID,joint1,targetPos1,vrep.simx_opmode_oneshot)

    #vrep.simxSetJointTargetVelocity(clientID,objs,targetVel,vrep.simx_opmode_oneshot)
    #vrep.rmlMoveToJointPositions(objs,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
    time.sleep(2)

    # Load joint 3
    res,joint3=vrep.simxGetObjectHandle(clientID,'UR3_joint3',vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID,joint3,targetPos1,vrep.simx_opmode_oneshot)
    time.sleep(2)

    res,joint5=vrep.simxGetObjectHandle(clientID,'UR3_joint5',vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID,joint5,-targetPos1,vrep.simx_opmode_oneshot)
    time.sleep(2)


    code,state,point,obj,normv = vrep.simxReadProximitySensor(clientID,posensor,vrep.simx_opmode_buffer)
    print(state)
    print(point)
    print(obj)
    print(normv)





    #ping pong ball
    res,ball=vrep.simxGetObjectHandle(clientID,'Sphere',vrep.simx_opmode_blocking)
    vrep.simxSetObjectFloatParameter(clientID,ball,3001,100,vrep.simx_opmode_streaming)


    ################################# don't modify beyond this line
    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
