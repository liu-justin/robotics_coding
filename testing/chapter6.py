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
    screw = ch3.se3_to_vector6(scriptV_b)
    (omega, v) = ch3.exp_coord6_extraction(screw)

    counter = 0

    while(ch3.magnitude(omega) > e_omega and ch3.magnitude(v) > e_v and counter > 25):
        thetaList = thetaList + np.dot(ch5.jacobian_body(Blist, thetaList), scriptV_b)
        # recalculate scriptV, screw and omega, and v
        T_sb = ch4.forward_kinematics_in_body(M, Blist, thetaList)
        T_bs = ch3.transf_matrix_inverse(T_sb)
        scriptV_b = ch3.transf_matrix_to_exp_coord6(T_bs @ T)
        screw = ch3.se3_to_vector6(scriptV_b)
        (omega, v) = ch3.exp_coord6_extraction(screw)
        counter += 1

    return (thetaList, counter < 25)