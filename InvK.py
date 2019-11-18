import numpy as np
from scipy.linalg import expm
import math
from MRpython import modern_robotics as mr

class InvK:
    def __init__(self,S_,T_):
        self.S = S_.transpose()
        self.M = []
        self.T = T_

    def find_M(self,endEffector):
        #rotate y by -90 degree
        R = np.array([[0,0, -1],[0,1,0],[1,0,0]])
        p = np.array([endEffector]).transpose()
        M_temp = np.concatenate([R,p],axis=1)
        bottom = np.array([[0,0,0,1]])
        M_temp = np.concatenate([M_temp,bottom])
        self.M = M_temp

    def find_thetas(self):
        thetas0 = [0,0,0,0,0,0]
        #using code from the code library accompanying 
        #Modern Robotics: Mechanics, Planning, and Control (Kevin Lynch and Frank Park, Cambridge University Press 2017)
        thetas,success = mr.IKinSpace(self.S,self.M,self.T,thetas0,0.01,0.01)
        print(success)
        return thetas if success else thetas0
