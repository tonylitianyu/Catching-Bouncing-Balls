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
from FK_calculation import FK_calculation
import numpy as np
from InvK import InvK
import random
import pickle



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
    targetPos1 = 50*math.pi/180
    targetVel = 0



    #sensor
    res,posensor=vrep.simxGetObjectHandle(clientID,'Basket',vrep.simx_opmode_blocking)
    #code,state,point,obj,normv = vrep.simxReadProximitySensor(clientID,posensor,vrep.simx_opmode_streaming) #sensor reading

    res, endEffectorPos = vrep.simxGetObjectPosition(clientID,posensor,-1,vrep.simx_opmode_streaming)# initialize sensor position
    res, endEffectorOri = vrep.simxGetObjectOrientation(clientID,posensor,-1,vrep.simx_opmode_streaming)# initialize sensor orientation


    # Load joint 1
    res,joint1=vrep.simxGetObjectHandle(clientID,'UR3_joint1',vrep.simx_opmode_blocking)
    code,posJ1 = vrep.simxGetObjectPosition(clientID,joint1,-1,vrep.simx_opmode_streaming)

    # Load joint 2
    res,joint2=vrep.simxGetObjectHandle(clientID,'UR3_joint2',vrep.simx_opmode_blocking)
    code,posJ2 = vrep.simxGetObjectPosition(clientID,joint2,-1,vrep.simx_opmode_streaming)

    #Load joint 3
    res,joint3=vrep.simxGetObjectHandle(clientID,'UR3_joint3',vrep.simx_opmode_blocking)
    code,posJ3 = vrep.simxGetObjectPosition(clientID,joint3,-1,vrep.simx_opmode_streaming)

    #Load joint 4
    res,joint4=vrep.simxGetObjectHandle(clientID,'UR3_joint4',vrep.simx_opmode_blocking)
    code,posJ4 = vrep.simxGetObjectPosition(clientID,joint4,-1,vrep.simx_opmode_streaming)

    # Load joint 5
    res,joint5=vrep.simxGetObjectHandle(clientID,'UR3_joint5',vrep.simx_opmode_blocking)
    code,posJ5 = vrep.simxGetObjectPosition(clientID,joint5,-1,vrep.simx_opmode_streaming)

    #Load joint 6
    res,joint6=vrep.simxGetObjectHandle(clientID,'UR3_joint6',vrep.simx_opmode_blocking)
    code,posJ6 = vrep.simxGetObjectPosition(clientID,joint6,-1,vrep.simx_opmode_streaming)


    time.sleep(1)

    res, endEffectorPos = vrep.simxGetObjectPosition(clientID,posensor,-1,vrep.simx_opmode_buffer)
    print('end effector initial position:')
    print(endEffectorPos)#get initial end effector position
    res, endEffectorOri = vrep.simxGetObjectOrientation(clientID,posensor,-1,vrep.simx_opmode_buffer)
    print('end effector initial orientation:')
    print(endEffectorOri)# initialize sensor orientation

    #get joint position
    code,posJ1 = vrep.simxGetObjectPosition(clientID,joint1,-1,vrep.simx_opmode_buffer)
    code,posJ2 = vrep.simxGetObjectPosition(clientID,joint2,-1,vrep.simx_opmode_buffer)
    code,posJ3 = vrep.simxGetObjectPosition(clientID,joint3,-1,vrep.simx_opmode_buffer)
    code,posJ4 = vrep.simxGetObjectPosition(clientID,joint4,-1,vrep.simx_opmode_buffer)
    code,posJ5 = vrep.simxGetObjectPosition(clientID,joint5,-1,vrep.simx_opmode_buffer)
    code,posJ6 = vrep.simxGetObjectPosition(clientID,joint6,-1,vrep.simx_opmode_buffer)




    #Initialize forward kinematics calculation by inputting target joint angles.
    jointAngles = np.array([targetPos1,0,0,0,0,0])

    w1 = np.array([0,0,1])
    #q1 = np.array([0.00012,0.000086,0.1475])
    q1 = np.array(posJ1)
    v1 = np.cross(-w1,q1)
    s1 = np.concatenate([w1,v1])


    w2 = np.array([-1,0,0])
    #q2 = np.array([-0.1115,0.000054,0.1519])
    q2 = np.array(posJ2)
    v2 = np.cross(-w2,q2)
    s2 = np.concatenate([w2,v2])

    w3 = np.array([-1,0,0])
    #q3 = np.array([-0.1115,0.00013,0.3955])
    q3 = np.array(posJ3)
    v3 = np.cross(-w3,q3)
    s3 = np.concatenate([w3,v3])

    w4 = np.array([-1,0,0])
    #q4 = np.array([-0.1115,0.000085,0.6088])
    q4 = np.array(posJ4)
    v4 = np.cross(-w4,q4)
    s4 = np.concatenate([w4,v4])

    w5 = np.array([0,0,1])
    #q5 = np.array([-0.1122,0.000085,0.6930])
    q5 = np.array(posJ5)
    v5 = np.cross(-w5,q5)
    s5 = np.concatenate([w5,v5])

    w6 = np.array([-1,0,0])
    #q6 = np.array([-0.1115,0.000085,0.6941])
    q6 = np.array(posJ6)
    v6 = np.cross(-w6,q6)
    s6 = np.concatenate([w6,v6])

    s = np.array([s1,s2,s3,s4,s5,s6])


    #ping pong ball
    #initialization of two balls
    res,ball0=vrep.simxGetObjectHandle(clientID,'Sphere',vrep.simx_opmode_blocking)
    res,ballpos = vrep.simxGetObjectPosition(clientID,ball0,-1,vrep.simx_opmode_streaming)
    time.sleep(1)
    res,ballOrigin = vrep.simxGetObjectPosition(clientID,ball0,-1,vrep.simx_opmode_buffer)
    time.sleep(1)

    # #make ball jump
    # ret_code, _, _, _, _ = vrep.simxCallScriptFunction(clientID, 'Sphere', vrep.sim_scripttype_childscript, 'shootBall', [], [], [], bytearray(), vrep.simx_opmode_blocking)


    res,xval=vrep.simxGetFloatSignal(clientID,"xtarget",vrep.simx_opmode_streaming)
    res,yval=vrep.simxGetFloatSignal(clientID,"ytarget",vrep.simx_opmode_streaming)
    res,zval=vrep.simxGetFloatSignal(clientID,"ztarget",vrep.simx_opmode_streaming)
    time.sleep(1)

    #while(1):
        #make ball jump
    #res,ballcurr = vrep.simxGetObjectPosition(clientID,ball0,-1,vrep.simx_opmode_buffer)
    #if ballcurr[0] > -0.2:
        # time.sleep(0.5)
        # res = vrep.simxSetObjectPosition(clientID,ball0,-1,ballOrigin,vrep.simx_opmode_oneshot)
        # time.sleep(1)

    ret_code, _, force, _, _ = vrep.simxCallScriptFunction(clientID, 'Sphere', vrep.sim_scripttype_childscript, 'shootBall', [], [], [], bytearray(), vrep.simx_opmode_blocking)
    shooting_angle = math.atan2(force[1],force[0])
    print('shooting angle')
    print(shooting_angle)

    #create shooting machine
    res,shootingMachine = vrep.simxGetObjectHandle(clientID,'Shooting',vrep.simx_opmode_blocking)
    res = vrep.simxSetObjectOrientation(clientID,shootingMachine,shootingMachine,[shooting_angle,0,0],vrep.simx_opmode_oneshot)



    time.sleep(0.5)
    res,xval=vrep.simxGetFloatSignal(clientID,"xtarget",vrep.simx_opmode_buffer)
    print('x:')
    print(xval)

    res,yval=vrep.simxGetFloatSignal(clientID,"ytarget",vrep.simx_opmode_buffer)
    print('y:')
    print(yval)

    res,zval=vrep.simxGetFloatSignal(clientID,"ztarget",vrep.simx_opmode_buffer)
    print('z:')
    print(zval)

    h_angle = math.atan2(yval,xval)
    print('angle')
    print(h_angle)

    target_x = -0.2
    if xval > 0.8:
        target_x = -0.15

    loaded_model = pickle.load(open('model.sav', 'rb'))
    result = loaded_model.predict([[xval,yval,zval]])
    target_x = result[0][0]
    print(loaded_model.coef_)
    print("x-result")

    target_x = -2.2433+0.25+xval*1.9804+zval*0.071423
    print(target_x)
    if target_x < -0.35:
        target_x = -0.2
    #1.975
    target_y = 1.975*math.tan(h_angle)
    target_z = 0.5

    FK = FK_calculation(jointAngles,s)
    #targetPosition = endEffectorPos
    targetPosition = np.array([target_x,target_y,target_z])
    M = FK.find_M(targetPosition) #M for forward kinematics
    T = FK.find_T()
    print('T from forward Kinematics: ')
    print(T)


    #inverse kinematics
    #using code from the code library accompanying
    #Modern Robotics: Mechanics, Planning, and Control (Kevin Lynch and Frank Park, Cambridge University Press 2017)
    IK = InvK(s,M) #T should be ball position, will implement next time
    IK.find_M(endEffectorPos)
    jointAngles = IK.find_thetas()
    print('joint angles from Inverse Kinematics: ')
    print(jointAngles)






    # Load joint 1
    #vrep.simxSetJointForce(clientID,joint1,3,vrep.simx_opmode_oneshot) #set max force for this joint
    vrep.simxSetJointTargetPosition(clientID,joint1,jointAngles[0],vrep.simx_opmode_oneshot)
    #time.sleep(1)

    # Load joint 2
    vrep.simxSetJointTargetPosition(clientID,joint2,jointAngles[1],vrep.simx_opmode_oneshot)
    #time.sleep(1)

    # Load joint 3
    vrep.simxSetJointTargetPosition(clientID,joint3,jointAngles[2],vrep.simx_opmode_oneshot)
    #time.sleep(1)

    # Load joint 4
    vrep.simxSetJointTargetPosition(clientID,joint4,jointAngles[3],vrep.simx_opmode_oneshot)
    #time.sleep(1)

    # Load joint 5
    vrep.simxSetJointTargetPosition(clientID,joint5,jointAngles[4],vrep.simx_opmode_oneshot)
    #time.sleep(1)

    # Load joint 6
    vrep.simxSetJointTargetPosition(clientID,joint6,jointAngles[5],vrep.simx_opmode_oneshot)
    #time.sleep(1)



    res, endEffectorPos = vrep.simxGetObjectPosition(clientID,posensor,-1,vrep.simx_opmode_buffer)
    print('end effector final position:')
    print(endEffectorPos)
    res, endEffectorOri = vrep.simxGetObjectOrientation(clientID,posensor,-1,vrep.simx_opmode_buffer)
    print('end effector final orientation:')
    print(endEffectorOri)











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
