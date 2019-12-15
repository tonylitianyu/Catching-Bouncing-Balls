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

    def guess_thetas(self):
        max_iter = 100
        max_err = 0.001
        mu = 0.05
        thetas = np.zeros(((self.S).shape[1], 1))
        V = np.ones((6, 1))
        while np.linalg.norm(V) > max_err and max_iter > 0:
            curr_pose = mr.FKinSpace(self.M, self.S, thetas)
            V = inv_skew(logm(np.dot(self.T,(inv(curr_pose)))))
            J = mr.JacobianSpace(self.S, thetas)
            pinv = inv(J.transpose().dot(J) + mu * np.identity((self.S).shape[1])).dot(J.transpose())
            thetadot = pinv.dot(V)
            thetas = thetas + thetadot
            max_iter -= 1;
        return (thetas, np.linalg.norm(V))

    def find_thetas(self):
        thetas0,norm = self.guess_thetas()
        #using code from the code library accompanying
        #Modern Robotics: Mechanics, Planning, and Control (Kevin Lynch and Frank Park, Cambridge University Press 2017)
        thetas,success = mr.IKinSpace(self.S,self.M,self.T,thetas0,0.01,0.01)
        print(success)
        return thetas if success else thetas0

"""helper function"""
def inv_skew(m):
    col = []
    if (m.shape == (4, 4)):
        col = np.concatenate((inv_skew(m[:3, :3]), m[:3, 3:]),axis=0)
    elif (m.shape == (3, 3)):
        col = np.zeros((3, 1))
        col[0] = m[2][1]
        col[1] = m[0][2]
        col[2] = m[1][0]
    return col