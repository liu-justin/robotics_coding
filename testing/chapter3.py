import numpy as np
import mpmath
import math
import time

def rotation_inverse(R):
    """Inverses a rotation matrix
    """
    # return np.array([[R[0][0], R[1][0], R[2][0]],
    #                  [R[0][1], R[1][1], R[2][1]],
    #                  [R[0][2], R[1][2], R[2][2]]])
    return np.transpose(R)

def so3_to_vector3(so3matrix):
    """converts an so(3) matrix to a 3vector
    Args:
        so3matrix: 3x3 real skew symmetric matrix
    """
    return np.array([so3matrix[2][1], -1*so3matrix[2][0], so3matrix[1][0]])

def vector3_to_so3(x):
    """converts a 3vector to so(3) 3x3 real skew symmetric matrix
    """
    return np.array([[    0, -x[2],  x[1]],
                     [ x[2],     0, -x[0]],
                     [-x[1],  x[0],     0]])

def exp_coord3_extraction(exp3):
    """extracts the omega and theta from exponential coord size 3

    Args:
        exp3: omega(axis of rotation) and theta multiplied together

    Returns:
        omega_hat: axis of rotation
        theta about the omega_hat 
    """
    theta = np.sqrt(exp3.dot(exp3))
    omega_hat = exp3/theta
    return (omega_hat, theta)

def so3_to_rotation_matrix(so3matrix):
    """converts a so3 into a matrix exponential, which equals a rotation matrix

    Args:
        so3matrix: omega_hat*theta

    Returns:
        rotation matrix
    """
    exp_coord3 = so3_to_vector3(so3matrix)
    (omega, theta) = exp_coord3_extraction(exp_coord3)
    first = np.identity(3)
    skewed_omega = vector3_to_so3(omega)
    second = math.sin(theta)*skewed_omega
    third = (1-math.cos(theta))*np.dot(skewed_omega,skewed_omega)

    return first+second+third

def omegatheta_to_rotation_matrix(omega, theta):
    first = np.identity(3)
    skewed_omega = vector3_to_so3(omega)
    second = math.sin(theta)*skewed_omega
    third = (1-math.cos(theta))*np.dot(skewed_omega,skewed_omega)

    return first+second+third
    
def rotation_matrix_to_exp_coord3(R):
    if ((R==np.identity(3)).all()):
        return ("undefined", 0)
    elif (np.trace(R)==-1):
        try:
            omega = 1/math.sqrt(2*(1+R[2][2]))*np.array([R[0][2], R[1][2], 1+R[2][2]])
        except:
            try:
                omega = 1/math.sqrt(2*(1+R[1][1]))*np.array([R[0][1], 1+R[1][1], R[2][1]])
            except:
                omega = 1/math.sqrt(2*(1+R[0][0]))*np.array([1+R[0][0], R[1][0], 1+R[2][0]])
        return (omega, np.pi)
    else:
        theta = math.acos((np.trace(R)-1)/2)
        omegahat = (R-np.transpose(R))/(2*math.sin(theta))
        return (theta, so3_to_vector3(omegahat))

def Rp_to_transf_matrix(R,p):
    addedCol = np.c_[R, p]
    return np.append(addedCol, [[0,0,0,1]], axis=0)

def transf_matrix_to_Rp(T):
    p = np.array([T[0][3], T[1][3], T[2][3]])
    R = np.delete(np.delete(T,3,1),3,0)
    return (R,p)

def transf_matrix_inverse(T):
    (R,p) = transf_matrix_to_Rp(T)
    R_new = rotation_inverse(R)
    p_new = -1*np.dot(R_new,p)
    return Rp_to_transf_matrix(R_new, p_new)

def vector6_to_se3(vector6):
    skewed_omega = vector3_to_so3([vector6[0], vector6[1], vector6[2]])
    v = np.array([vector6[3], vector6[4], vector6[5]])
    addedCol = np.c_[skewed_omega,v]
    return np.append(addedCol, [[0,0,0,0]], axis=0)

def se3_to_vector6(matrix):
    (angular, linear) = transf_matrix_to_Rp(matrix)
    omega = so3_to_vector3(angular)
    return np.append(omega, linear)

def adjoint(T):
    (R,p) = transf_matrix_to_Rp(T)
    p_skewed = vector3_to_so3(p)
    top_half = np.append(R, np.zeros((3,3)), axis=1)
    bottom_half = np.append(np.dot(p_skewed,R), R, axis=1)
    return np.append(top_half, bottom_half, axis=0)

def qsh_to_screwaxis(q,s,h):
    omega = s
    v = np.cross(-1*s, q) + h*s
    return np.append(omega, v, axis=0)

# exponential coordinates (6) is the screw axis * theta, which is a twist
def exp_coord6_extraction(exp6):
    theta = np.sqrt(exp6.dot(exp6))
    screwaxis = exp6/theta
    return (screwaxis, theta)

def twist_extraction(twist):
    exp_coord6_extraction(twist)

# se3 is set of all 4x4 matrices of form skewed omega and v
# matrix exponential of se3 is just the left side in Prop 3.25
def matrixExp_to_transf_matrix(*args): 
    if len(args) == 1: # this se3 comes with screw axis and theta combined, need to seperate
        exp_coord6 = se3_to_vector6(args[0])
        (screwaxis, theta) = exp_coord6_extraction(exp_coord6)
    else: # screw axis and theta are seperated
        screwaxis = args[0]
        theta = args[1]
    omega = np.delete(screwaxis,[3,4,5], axis=0)
    v = np.delete(screwaxis,[0,1,2], axis=0)
    # print(f"screwaxis: \n{screwaxis}")
    # print(f"omega: \n{omega}")
    # print(f"v: \n{v}")
    skewed_omega = vector3_to_so3(omega)
    upleft = omegatheta_to_rotation_matrix(omega, theta)
    # print(f"upleft: \n{upleft}")
    upright = np.dot(theta*np.identity(3) + (1-math.cos(theta))*skewed_omega + (theta-math.sin(theta))*np.dot(skewed_omega, skewed_omega), v)
    # print(f"upright: \n{upright}")
    top = np.append(upleft, np.transpose(np.array([upright])), axis=1)
    return np.append(top, np.array([[0,0,0,1]]), axis=0)

def magnitude(vector):
    # returns the magnitude of any sized vector
    return math.sqrt(vector.dot(vector))

def transf_matrix_to_exp_coord6(T):
    (R,p) = transf_matrix_to_Rp(T)
    if ((R==np.identity(3)).all()):
        omega = 0
        v = p/magnitude(p)
        theta = magnitude(p)

    else:
        (theta, omega) = rotation_matrix_to_exp_coord3(R)
        skewed_omega = vector3_to_so3(omega)
        G_inverse = np.identity(3)/theta - skewed_omega/2 + (1/theta - (1/math.tan(theta/2))/2)*np.dot(skewed_omega, skewed_omega)
        v = np.dot(G_inverse, p)

    return (np.append(omega, v, axis=0), theta)

def Ex3_48 (T, q, s, h, theta):
    screwaxis = qsh_to_screwaxis(q,s,h)
    S_theta = vector6_to_se3(screwaxis)*theta
    matrixExp = matrixExp_to_transf_matrix(S_theta)
    return np.dot(matrixExp, T)

# Ra = np.array([[0,1,0],
#                [1,0,0],
#                [0,0,1]])
# p = np.array([4,5,8])

omega = np.array([0,0,1])
print(vector3_to_so3(omega))

# T = Rp_to_transf_matrix(Ra, p)
# print(f"this is the transformation matrix from Ra and p: \n{T}")

# (Rnew,pnew) = transf_matrix_to_Rp(T)
# print(f"this is the above transf matrix broken back into R and p: \n{Rnew}\n{pnew}")

# T_inv = transf_matrix_inverse(T)
# print(f"This is the transf matrix inversed: \n{T_inv}")

twist = np.array([0,2,2,4,0,0])
# se3ed_twist = vector6_to_se3(twist)
# print(f"this is the 6vector/twist [0,2,2,4,0,0] in se(3) form: \n{se3ed_twist}")
# back_to_twist = se3_to_vector6(se3ed_twist)
# print(f"and this is that se(3) matrix back to the twist: \n{back_to_twist}")

# # adjT = adjoint(T)
# # print(adjT)

R = np.identity(3) 
p = np.array([0,0,0])
T = Rp_to_transf_matrix(R,p)
adjT = adjoint(T)
print(adjT)
print(np.dot(adjT, twist))


# (screw1, theta1) = exp_coord6_extraction(twist)

# T1 = matrixExp_to_transf_matrix(screw1, np.pi/5)
# print(T1)

# T2 = matrixExp_to_transf_matrix(np.array([0,1,0,1,0,0]), np.pi/4)
# print(T2)

# aStart = time.perf_counter()
# a = rotationInverse(Ra)
# aEnd = time.perf_counter()
# print(aStart-aEnd)

# bStart = time.perf_counter()
# b = np.transpose(Ra)

# bEnd = time.perf_counter()
# print(bStart-bEnd)