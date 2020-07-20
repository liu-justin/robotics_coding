import numpy as np
import math
import chapter3 as ch3
import chapter4 as ch4

def problem4_2():
    R = np.identity(3)
    p = np.array([0,2,1])
    M = ch3.Rp_to_transf(R,p)
    thetaList = [np.pi/2,np.pi/2, -1*np.pi/2, 1]
    screwSpaceAxis1 = ch3.qsh_to_screwaxis(np.array([0,0,0]), np.array([0,0,1]), 0)
    screwSpaceAxis2 = ch3.qsh_to_screwaxis(np.array([0,1,0]), np.array([0,0,1]), 0)
    screwSpaceAxis3 = ch3.qsh_to_screwaxis(np.array([0,2,0]), np.array([0,0,1]), 0)
    screwSpaceAxis4 = np.array([0,0,0,0,0,1])

    # screwBodyAxis1 = np.dot(ch3.adjoint(ch3.transf_inverse(M)), screwSpaceAxis1)

    screwSpaceList = [screwSpaceAxis1, screwSpaceAxis2, screwSpaceAxis3, screwSpaceAxis4]
    screwBodyList = []
    for screwSpaceAxis in screwSpaceList:
        screwBodyList.append(np.dot(ch3.adjoint(ch3.transf_inverse(M)), screwSpaceAxis))

    print(screwBodyList)

    FK_space = ch4.forward_kinematics_in_space(M, screwSpaceList, thetaList)
    print(FK_space)

    FK_body = ch4.forward_kinematics_in_body(M, screwBodyList, thetaList)
    print(FK_body)

def problem4_17():
    xaxis = np.array([1,0,0])
    R = np.dot(ch3.omegatheta_to_rotation(xaxis, np.pi/2), np.identity(3))
    p = np.array([4,1,1])
    M = ch3.Rp_to_transf(R,p)
    S3 = ch3.qsh_to_screwaxis(np.array([1,0,1]), np.array([0,-1,0]), 0)    
    S5 = ch3.qsh_to_screwaxis(np.array([3,0,1]), np.array([0,-1,0]), 0)

    # i dont know what is happening in the next part, so im just skipping

problem4_17()