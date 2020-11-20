import modules.unpack as unpack
import modules.modern_robotics as mr
import numpy as np

T_ee, T_list, body_list, G_list = unpack.unpack_XML("6DoF_URDF.xml")

theta_home = np.array([0,-1*np.pi/2, np.pi/2,0,0,0])

M_home = mr.FKinBody(T_ee, body_list, theta_home)

M_test = np.array([[1,0,0,0.25],
                  [0,1,0,0.1],
                  [0,0,1,0],
                  [0,0,0,1]])

Tf = 3 # total time for move
N = 50 # number of times to sample
a = unpack.rest_to_home_angle_list()
print(a)

unpack.angleList_push(a,1)

#angle_list = unpack.movement_to_angleList(M_home, theta_home, M_test, 3, 50, body_list, T_ee)
# print(angle_list)