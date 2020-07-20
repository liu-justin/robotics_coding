import numpy as np
import chapter3 as ch3

def forward_kinematics_in_body(M, screwBodyList, thetaList):
    end_effector_frame = np.identity(4)
    revScrewList = [ele for ele in reversed(screwBodyList)]
    revThetaList = [ele for ele in reversed(thetaList)]
    for screw, theta in zip(revScrewList, revThetaList):
        matrixExp = ch3.matrixExp_to_transf(screw, theta)
        end_effector_frame = np.dot(matrixExp, end_effector_frame)
    
    end_effector_frame = np.dot(M, end_effector_frame)
    return np.round(end_effector_frame,5)

def forward_kinematics_in_space(M, screwSpaceList, thetaList):
    end_effector_frame = M
    revScrewList = [ele for ele in reversed(screwSpaceList)]
    revThetaList = [ele for ele in reversed(thetaList)]
    for screw, theta in zip(revScrewList, revThetaList):
        matrixExp = ch3.matrixExp_to_transf(screw, theta)
        end_effector_frame = np.dot(matrixExp, end_effector_frame)
    return np.round(end_effector_frame,5)
