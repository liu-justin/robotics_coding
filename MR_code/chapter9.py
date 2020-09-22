import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5
import chapter6 as ch6

import modern_robotics_functions as mr

import math
import numpy as np

def cubic_time_scaling(Tf, t):
    a2 = 3/(T**2)
    a3 = -2/(T**2)
    return a2*t**2 + a3*t**3

def quintic_time_scaling(Tf, t):
    return 10 * (1.0 * t / Tf) ** 3 - 15 * (1.0 * t / Tf) ** 4 + 6 * (1.0 * t / Tf) ** 5

def joint_trajectory(theta_start, theta_end, Tf, N, method):
    """Computes a straight-line trajectory in joint space
    :param thetastart: The initial joint variables
    :param thetaend: The final joint variables
    :param Tf: Total time of the motion in seconds from rest to rest
    :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
    :param method: The time-scaling method, where 3 indicates cubic (third-
                   order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
    :return: A trajectory as an N x n matrix, where each row is an n-vector
             of joint variables at an instant in time. The first row is
             thetastart and the Nth row is thetaend . The elapsed time
             between each row is Tf / (N - 1)"""

    N = int(N)
    timegap = Tf / (N - 1.0) # N points, N-1 line segments
    traj = np.zeros((len(theta_start), N)) # intitialize the trajectory matrix, 1D joint vars, 2D each time instance

    # for each line segment, from 0 to T, calculate the corresponding s value (0to1)
    for i in range(N):
        if method == 3:
            s = cubic_time_scaling(Tf, timegap * i)
        else:
            s = quintic_time_scaling(Tf, timegap * i)
        traj[:, i] = s * np.array(theta_end) + (1 - s) * np.array(theta_start) # xi = x_start + (0.whatever fraction s)(x_end-x_start)
    traj = np.array(traj).T
    return traj

def screw_trajectory(X_start, X_end, Tf, N, method):
    """Computes a trajectory as a list of N SE(3) matrices corresponding to
      the screw motion about a space screw axis (each matrix represents the config of the end effector at an instant in time)
    :param Xstart: The initial end-effector configuration
    :param Xend: The final end-effector configuration
    :param Tf: Total time of the motion in seconds from rest to rest
    :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
    :param method: The time-scaling method, where 3 indicates cubic (third-
                   order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
    :return: The discretized trajectory as a list of N matrices in SE(3)
             separated in time by Tf/(N-1). The first in the list is Xstart
             and the Nth is Xend"""
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N

    X_start_end = np.dot(ch3.transf_matrix_inverse(X_start), X_end)
    for i in range(N):
        if method == 3:
            s = cubic_time_scaling(Tf, timegap * i)
        else:
            s = quintic_time_scaling(Tf, timegap * i)

        fractioned_X_start_end = ch3.se3_to_transf_matrix(ch3.transf_matrix_to_se3(X_start_end) * s) # applying s to the se3 matrix instead of the transf matrix, it works
        traj[i] = np.dot(X_start, fractioned_X_start_end)
    return traj

def cartesian_trajectory(X_start, X_end, Tf, N, method):
    """Computes a trajectory as a list of N SE(3) matrices corresponding to
    the origin of the end-effector frame following a straight line
    :param Xstart: The initial end-effector configuration
    :param Xend: The final end-effector configuration
    :param Tf: Total time of the motion in seconds from rest to rest
    :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
    :param method: The time-scaling method, where 3 indicates cubic (third-
                   order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
    :return: The discretized trajectory as a list of N matrices in SE(3)
             separated in time by Tf/(N-1). The first in the list is Xstart
             and the Nth is Xend
    This function is similar to ScrewTrajectory, except the origin of the
    end-effector frame follows a straight line, decoupled from the rotational
    motion."""

    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    R_start, p_start = ch3.transf_matrix_to_Rp(X_start)
    R_end, p_end = ch3.transf_matrix_to_Rp(X_end)

    R_start_end = np.dot(np.array(R_start).T,R_end)
    for i in range(N):
        if method == 3:
            s = cubic_time_scaling(Tf, timegap * i)
        else:
            s = quintic_time_scaling(Tf, timegap * i)

        R_start_end_fractioned = ch3.so3_to_rotation_matrix(ch3.rotation_matrix_to_so3(R_start_end) * s)
        R_start_s = np.dot(R_start, R_start_end_fractioned)

        p_start_s = s * np.array(p_end) + (1 - s) * np.array(p_start) #p_start + s(p_end-p_start)
        traj[i] = np.r_[np.c_[R_start_s, p_start_s], \
                            [[  0, 0, 0,         1]]]

                   
    return traj