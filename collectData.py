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
from sklearn.neighbors import KNeighborsRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.linear_model import LinearRegression
import pickle
import pandas as pd


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    input = []
    output = []

    #ping pong ball
    #initialization of ball
    res,ball0=vrep.simxGetObjectHandle(clientID,'Sphere',vrep.simx_opmode_blocking)
    res,ballpos = vrep.simxGetObjectPosition(clientID,ball0,-1,vrep.simx_opmode_streaming)
    time.sleep(1)
    res,ballOrigin = vrep.simxGetObjectPosition(clientID,ball0,-1,vrep.simx_opmode_buffer)
    time.sleep(1)


    res,xval=vrep.simxGetFloatSignal(clientID,"xtarget",vrep.simx_opmode_streaming)
    res,yval=vrep.simxGetFloatSignal(clientID,"ytarget",vrep.simx_opmode_streaming)
    res,zval=vrep.simxGetFloatSignal(clientID,"ztarget",vrep.simx_opmode_streaming)

    res,xdest=vrep.simxGetFloatSignal(clientID,"xdest",vrep.simx_opmode_streaming)
    res,ydest=vrep.simxGetFloatSignal(clientID,"ydest",vrep.simx_opmode_streaming)
    res,zdest=vrep.simxGetFloatSignal(clientID,"zdest",vrep.simx_opmode_streaming)
    time.sleep(1)
    count = 0
    while count < 500:
        count += 1
        # #make ball jump
        ret_code, _, _, _, _ = vrep.simxCallScriptFunction(clientID, 'Sphere', vrep.sim_scripttype_childscript, 'shootBall', [], [], [], bytearray(), vrep.simx_opmode_blocking)

        time.sleep(0.5)
        #speed
        res,xval=vrep.simxGetFloatSignal(clientID,"xtarget",vrep.simx_opmode_buffer)
        #print('x:')
        #print(xval)

        res,yval=vrep.simxGetFloatSignal(clientID,"ytarget",vrep.simx_opmode_buffer)
        #print('y:')
        #print(yval)

        res,zval=vrep.simxGetFloatSignal(clientID,"ztarget",vrep.simx_opmode_buffer)
        #print('z:')
        #print(zval)

        time.sleep(3)

        #second bounce place
        res,xdest=vrep.simxGetFloatSignal(clientID,"xdest",vrep.simx_opmode_buffer)
        res,ydest=vrep.simxGetFloatSignal(clientID,"ydest",vrep.simx_opmode_buffer)
        res,zdest=vrep.simxGetFloatSignal(clientID,"zdest",vrep.simx_opmode_buffer)

        print(str(count))
        print(xdest)
        # vel = [xval,yval,zval]
        # dest = [xdest,ydest,zdest]

        input.append([xval,yval,zval])
        output.append(xdest)
        time.sleep(1)
        res = vrep.simxSetObjectPosition(clientID,ball0,-1,ballOrigin,vrep.simx_opmode_oneshot)
        time.sleep(1)


    print(input)
    print(output)


    # knn = KNeighborsRegressor()
    # regressor = MultiOutputRegressor(knn)
    # regressor.fit(input,output)
    # filename = 'model.sav'
    # pickle.dump(regressor, open(filename, 'wb'))
    # ans = regressor.predict([[0.9369611740112305]])
    # print(ans)#-0.0031253783963620663
    csvArr = np.concatenate((np.array(input), np.array([output]).T), axis=1)
    pd.DataFrame(csvArr).to_csv("regre.csv")


    reg = LinearRegression().fit(np.array(input), np.array(output).reshape(-1, 1))
    filename = 'model.sav'
    pickle.dump(reg, open(filename, 'wb'))
    ans = reg.predict([[0.9369611740112305,-0.21374374628067017,6.679958343505859]])
    print(ans)#-0.0031253783963620663
    print(reg.coef_)

    #-2.75873539



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
