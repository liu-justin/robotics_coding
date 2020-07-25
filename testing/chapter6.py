import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5 

import math
import numpy as np

def IK_body(Blist, M, T, thetalist0, e_omega, e_v):
    """uses iterative Netwon Raphson to calculate inverse kinematics

    Args:
        Blist : list of joint screws expressed in the end effector frame scriptB
        M : end effector home configuration
        T : desired end effector configuration
        thetalist0: initial guess
        e_omega : omega error tolerance for stoppage
        e_v : linear velocity error tolerance for stoppage
    """
    thetaList = thetalist0
    T_sb = ch4.forward_kinematics_in_body(M, Blist, thetaList)
    T_bs = ch3.transf_matrix_inverse(T_sb)
    (scriptV_b, theta) = ch3.transf_matrix_to_se3(T_bs @ T)
    scriptV_b_exp6 = ch3.se3_to_vector6(scriptV_b)
    omega = scriptV_b_exp6[0:3]
    v = scriptV_b_exp6[3:6]

    counter = 0
    print(ch3.magnitude(omega))
    print(ch3.magnitude(v))
    while((ch3.magnitude(omega) > e_omega or ch3.magnitude(v) > e_v) and counter < 25):
        jacobian_b = ch5.jacobian_body(Blist, thetaList)
        print(f"this is the jacobian, should be 6x2: {jacobian_b}")
        thetaListaddition = np.dot(jacobian_b.T, np.reshape(scriptV_b_exp6, (-1,1)))
        thetaList = [sum(i) for i in zip(thetaList,thetaListaddition.flatten().tolist())]
        # recalculate scriptV, screw and omega, and v
        T_sb = ch4.forward_kinematics_in_body(M, Blist, thetaList)
        T_bs = ch3.transf_matrix_inverse(T_sb)
        (scriptV_b, theta) = ch3.transf_matrix_to_se3(T_bs @ T)
        scriptV_b_exp6 = ch3.se3_to_vector6(scriptV_b)
        omega = scriptV_b_exp6[0:3]
        v = scriptV_b_exp6[3:6]


        counter += 1


    return (thetaList, counter < 25)