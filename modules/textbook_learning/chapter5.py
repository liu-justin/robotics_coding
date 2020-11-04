import numpy as np
import chapter3 as ch3
import chapter4 as ch4

def jacobian_space(screwSpaceList, thetaList):
    """pulling from eq 5.11 in MRv2, this equation makes use of the adjoint function in ch3
    to create the space jacobian
    """
    jacobian = np.array([[1],[1],[1],[1],[1],[1]])
    scalingMatrixExp = np.identity(4)

    for screw, theta in zip(screwSpaceList, thetaList):
        
        column = np.array([np.dot(ch3.adjoint_transf_matrix(scalingMatrixExp),screw)])
        jacobian = np.append(jacobian, np.transpose(column), axis=1)

        matrixExp = ch3.screwtheta_to_transf_matrix(screw, theta)
        scalingMatrixExp = np.dot(scalingMatrixExp, matrixExp)

    jacobian = np.delete(jacobian,0,axis=1)
    return np.round(jacobian, 5)

def jacobian_body(screwBodyList, thetaList):

    revScrewList = [ele for ele in reversed(screwBodyList)]
    revThetaList = [ele for ele in reversed(thetaList)]

    jacobian = np.array([[1],[1],[1],[1],[1],[1]])
    scalingMatrixExp = np.identity(4)

    for screw, theta in zip(revScrewList, revThetaList):
        
        column = np.array([np.dot(ch3.adjoint_transf_matrix(scalingMatrixExp),screw)])
        jacobian = np.append(np.transpose(column), jacobian, axis=1)

        matrixExp = ch3.screwtheta_to_transf_matrix(screw, -1* theta)
        scalingMatrixExp = np.dot(scalingMatrixExp, matrixExp)

    # stupid, count starts at 1
    jacobian = np.delete(jacobian,len(screwBodyList),axis=1)
    return jacobian