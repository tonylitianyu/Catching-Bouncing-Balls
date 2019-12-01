import numpy as np
from scipy.linalg import expm, logm
from numpy.linalg import inv
import math
from MRpython import modern_robotics as mr
import FK_calculation

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

    def guess_thetas(self,endEffector):
        max_iter = 100
        max_err = 0.001
        mu = 0.05
        thetas = np.zeros(((self.S).shape[1], 1))
        V = np.ones((6, 1))
        while np.linalg.norm(V) > max_err and max_iter > 0:
            # fk = FK_calculation(thetas,self.S)
            # fk.M = self.M
            # curr_pose = fk.find_T()
            curr_pose = mr.FKinSpace(self.M, self.S, thetas)
            V = inv_bracket(logm(endEffector.dot(inv(curr_pose))))
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
def inv_bracket(m):
    rtn = []
    m = np.asarray(m)
    if (m.shape == (4, 4)):
        rtn = np.block([[inv_bracket(m[:3, :3])],
                        [m[:3, 3:]]])
    elif (m.shape == (3, 3)):
        m = m - m.transpose()
        rtn = np.zeros((3, 1))
        rtn[2] = - m[0][1] / 2
        rtn[1] = m[0][2] / 2
        rtn[0] = - m[1][2] / 2
    return rtn

# def skew4(V_b):
#     return np.array([[0, -1 * V_b[2], V_b[1], V_b[3]], [V_b[2], 0, -1 * V_b[0], V_b[4]], [-1 * V_b[1], V_b[0], 0, V_b[5]], [0, 0, 0, 0]])
#
# def evalJ(S, theta):
#     T = [expm(skew4 * theta[i]) for i in range(S.shape[1])]
#     J = S[:, [0]]
#     for i in range(1, S.shape[1]):
#         col = T[0]
#         for j in range(1, i):
#             col = col.dot(T[j])
#         newterm = adj_T(col).dot(S[:, [i]])
#         J = np.concatenate((J, newterm), axis=1)
#     return J