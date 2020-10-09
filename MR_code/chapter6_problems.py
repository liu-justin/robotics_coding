import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5 
import chapter6 as ch6

import numpy as np
import math

# R = np.identity(3) 
# p = np.array([2,0,0])
# M = ch3.Rp_to_transf_matrix(R,p)

# bodyScrew1 = np.array([0,0,1,0,2,0])
# bodyScrew2 = np.array([0,0,1,0,1,0])
# bodyList = [bodyScrew1, bodyScrew2]

# T = np.array([[-0.5, -0.866, 0, 0.366],
#               [0.866, -0.5, 0 ,1.366],
#               [0,0,1,0],
#               [0,0,0,1]])

# thetaList0 = [0, np.pi/6]

R = np.identity(3)
p = np.array([-0.041718, 0.239625, 0])
M = ch3.Rp_to_transf_matrix(R,p)

R3 = np.array([1, 0, 0,     0,       0,      0])
T3 = np.array([0,-1, 0,     0,-0.02328,      0])
R2 = np.array([1, 0, 0,     0,       0,      0])
T2 = np.array([0, 0,-1,0.0625,-0.20828,      0])
T1 = np.array([0, 0,-1,0.0625, 0.04172,      0])
R1 = np.array([0, 0, 1,0     ,       0,0.04172])
bodyList = [R1, T1, T2, R2, T3, R3]
# bodyList = [R1, T1, T2, R2]

thetaList0 = [0, np.pi/6, 0, 0, 0, 0]

p = np.array([0, 0.25, 0])
T = ch3.Rp_to_transf_matrix(R,p)
print(T)

e_omega = 0.001
e_velocity = 0.0001

T_sb = ch4.forward_kinematics_in_body(M, bodyList, thetaList0)
print(T_sb)

(final_angles, success) = ch6.IK_body(bodyList, M, T, thetaList0, e_omega, e_velocity)
print(final_angles)
print(success)

