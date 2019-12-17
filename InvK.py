import numpy as np
from scipy.linalg import expm, logm
from numpy.linalg import inv
import math
from MRpython import modern_robotics as mr
from FK_calculation import FK_calculation

class InvK:
    def __init__(self,S_,T_):
        self.S = S_.transpose()
        self.M = []
        self.T = T_

    def find_M(self,endEffector):
        #rotate y by -90 degree
        R = np.array([[0,-1, 0],[1,0,0],[0,0,1]])
        p = np.array([endEffector]).transpose()
        M_temp = np.concatenate([R,p],axis=1)
        bottom = np.array([[0,0,0,1]])
        M_temp = np.concatenate([M_temp,bottom])
        self.M = M_temp


    def newton_theta(self):
        count = 0
        theta = [[0],[0],[0],[0],[0],[0]]
        #V = np.zeros((6,1))
        diff = 1
        while diff > 0.01 and count < 100:
            currT = mr.FKinSpace(self.M, self.S, theta)
            V_b = logm(self.T@inv(currT))
            V = np.zeros((6,1))
            R = V_b[:3,:3]
            p = V_b[:3,3:]
            V[0] = R[2][1]
            V[1] = R[0][2]
            V[2] = R[1][0]
            V[3] = p[0]
            V[4] = p[1]
            V[5] = p[2]
            J = mr.JacobianSpace(self.S, theta)
            pinv = inv(J.T@J+ 0.01 * np.identity(6))@J.T
            theta = theta+pinv@V
            count = count+1
            diff = np.linalg.norm(V)
        return theta


    def find_thetas(self):

        thetas0 = self.newton_theta()
        #using code from the code library accompanying
        #Modern Robotics: Mechanics, Planning, and Control (Kevin Lynch and Frank Park, Cambridge University Press 2017)
        thetas,success = mr.IKinSpace(self.S,self.M,self.T,thetas0,0.01,0.01)

        return thetas if success else thetas0
