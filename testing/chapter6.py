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
        e_v : linear veloicty error tolerance for stoppage
    """
    thetaList = thetalist0
    scriptV_b = math.log(np.dot(ch4.forward_kinematics_in_body(M,Blist, thetalist0), T))
    screw = ch3.se3_to_vector(scriptV_b)
    (omega, v) = ch3.exp_coord6_extraction(screw)
    while((ch3.magnitude(omega) > e_omega or ch3.magnitude(v) > e_v)):
        thetaList = thetaList + np.dot(ch5.jacobian_body(Blist, thetaList), scriptV_b)
        # recalculate scriptV, screw and omega, and v