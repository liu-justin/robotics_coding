import numpy as np
import math
import chapter3 as ch3
import chapter4 as ch4
import chapter5 as ch5

def testingJacobian():
    R = np.identity(3)
    p = np.array([0,2,1])
    M = ch3.Rp_to_transf_matrix(R,p)
    thetaList = [np.pi/2,np.pi/2, -1*np.pi/2, 1]
    screwSpaceAxis1 = ch3.qsh_to_screwaxis(np.array([0,0,0]), np.array([0,0,1]), 0)
    screwSpaceAxis2 = ch3.qsh_to_screwaxis(np.array([0,1,0]), np.array([0,0,1]), 0)
    screwSpaceAxis3 = ch3.qsh_to_screwaxis(np.array([0,2,0]), np.array([0,0,1]), 0)
    screwSpaceAxis4 = np.array([0,0,0,0,0,1])

    screwSpaceList = [screwSpaceAxis1, screwSpaceAxis2, screwSpaceAxis3, screwSpaceAxis4]
    screwBodyList = []
    for screwSpaceAxis in screwSpaceList:
        screwBodyList.append(np.dot(ch3.adjoint(ch3.transf_matrix_inverse(M)), screwSpaceAxis))

    FK_space = ch5.jacobian_space(screwSpaceList, thetaList)
    print(FK_space)

    # FK_body = ch4.forward_kinematics_in_body(M, screwBodyList, thetaList)
    # print(FK_body)
    
def testingJacobianWithExample():
    R = np.identity(3)
    p = np.array([0,2,1])
    M = ch3.Rp_to_transf(R,p)
    thetaList = [0,0, 1*np.pi/2, 1]
    screwSpaceAxis1 = np.array([0,0,1,0,0,0])
    screwSpaceAxis2 = np.array([0,0,1,2*math.sin(thetaList[0]),-2*math.cos(thetaList[0]),0])
    screwSpaceAxis3 = np.array([0,0,1,2*math.sin(thetaList[0])+3*math.sin(thetaList[1]),-2*math.cos(thetaList[0])- 3*math.cos(thetaList[1]),0])
    screwSpaceAxis4 = np.array([0,0,0,0,0,1])

    screwSpaceList = [screwSpaceAxis1, screwSpaceAxis2, screwSpaceAxis3, screwSpaceAxis4]
    screwBodyList = []
    for screwSpaceAxis in screwSpaceList:
        screwBodyList.append(np.dot(ch3.adjoint(ch3.transf_inverse(M)), screwSpaceAxis))

    FK_space = ch5.jacobian_space(screwSpaceList, thetaList)
    print(FK_space)

    # FK_body = ch4.forward_kinematics_in_body(M, screwBodyList, thetaList)
    # print(FK_body)
testingJacobianWithExample()

# most stuff is in the notebook