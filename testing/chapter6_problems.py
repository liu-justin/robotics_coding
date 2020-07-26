import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5 
import chapter6 as ch6

import numpy as np
import math

R = np.identity(3) 
p = np.array([2,0,0])
M = ch3.Rp_to_transf_matrix(R,p)

bodyScrew1 = np.array([0,0,1,2,0,0])
bodyScrew2 = np.array([0,0,1,0,1,0])
bodyList = [bodyScrew1, bodyScrew2]

thetaList0 = [0, np.pi/6]

T = np.array([[-0.5, -0.866, 0, 0.366],
              [0.866, -0.5, 0 ,1.366],
              [0,0,1,0],
              [0,0,0,1]])

e_omega = 0.001
e_velocity = 0.0001

(final_angles, success) = ch6.IK_body(bodyList, M, T, thetaList0, e_omega, e_velocity)