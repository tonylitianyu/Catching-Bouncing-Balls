import numpy as np
from scipy.linalg import expm
import math

class FK_calculation:
    def __init__(self,jointAngles,s):
        self.angles = jointAngles
        self.M = []

        self.s = s

    def find_M(self,init_translation):
        #rotate y by -90 degree
        angle = -90*math.pi/180
        R = np.array([[0,-1, 0],[1,0,0],[0,0,1]])#np.array([[0,-1, 0],[1,0,0],[0,0,1]])

        p = np.array([init_translation]).transpose()
        M_temp = np.concatenate([R,p],axis=1)
        bottom = np.array([[0,0,0,1]])
        M_temp = np.concatenate([M_temp,bottom])
        self.M = M_temp

        return self.M

    def find_skew_W(self,w):
        result = np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
        return result


    def find_skew_S(self,s):
        w = s[0:3]

        v = np.array([s[3:6]]).transpose()

        skew_w = np.array(self.find_skew_W(w))


        w_and_v = np.concatenate([skew_w,v],axis=1)
        zeroBottom = np.array([np.zeros(4)])
        result = np.concatenate([w_and_v,zeroBottom])

        return result


    def find_T(self):

        skew_s1 = self.find_skew_S(self.s[0])

        T = expm(skew_s1*self.angles[0])

        for i in range(1,6):

            skew_s = self.find_skew_S(self.s[i])
            T = T@expm(skew_s*self.angles[i])

        T = T@self.M
        return T
