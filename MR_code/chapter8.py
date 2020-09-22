import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5
import chapter6 as ch6

import modern_robotics_functions as mr

import math
import numpy as np
import itertools

def adjoint_twist(V):
    omega_skew = ch3.vector3_to_so3([V[0], V[1], V[2]])
    v_skew = ch3.vector3_to_so3([V[3], V[4], V[5]])

    zero_init = (3,3)
    zero = np.zeros(zero_init)

    top = np.append(omega_skew, zero, axis=1)
    bot = np.append(v_skew, omega_skew, axis=1)

    return np.append(top, bot, axis=0)

def inverse_dynamics_bad(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList):
    forces = [np.array(fTip)]
    torques = []
    transfs = []
    twists = [np.array([0,0,0,0,0,0])] # first twist shouldn't be moving if arm is stationary, if attached to a wheeled platform then yeah change it
    twistDots = [np.append(np.zeros(3),-1*g)]

    # have to edit both for loops for i=1..n, i=n...1 (cut some lists up) hard

    for (theta, dtheta, ddtheta, M, S) in zip(thetaList, d_thetaList, dd_thetaList, MList[1:], np.transpose(SList)):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        A = np.dot(ch3.adjoint_transf_matrix(ch3.transf_matrix_inverse(Mi)), np.array(S_list)[:,i])
        T = ch3.screwtheta_to_transf_matrix(A,theta) @ M
        V = np.dot(ch3.adjoint_transf_matrix(T),twists[-1]) + A*dtheta
        Vdot = np.dot(ch3.adjoint_transf_matrix(T), twistDots[-1]) + np.dot(adjoint_twist(V),A)*dtheta + A*ddtheta 

        transfs.append(T)
        twists.append(V)
        twistDots.append(Vdot)

    # twists and twistDots contain the zeroth twist and twistDot, but this zip will stop at the smallest list end, so it fine
    for (twist, twistDot, G, A) in zip(twists[::-1], twistDots[::-1], GList[::-1], np.transpose(SList)[::-1]):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        first = np.dot(ch3.adjoint_transf_matrix(T),forces[0])
        second = np.dot(G,twistDot)
        third = np.dot(adjoint_twist(V),(np.dot(G,twist)))
        force = first+second-third
        tau = np.dot(force,A)

        # np.column_stack((forces,force))
        forces.insert(0,force)
        torques.insert(0,tau)

    return torques

def inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list):
    n = len(thetaList)

    # initializing the twist, and twist dot
    Vi = np.zeros((6,n+1)) # if there are 3 joints, then there are 4 twists to find (n+1)
    Vdi = np.zeros((6,n+1))
   
    Vdi[:,0] = np.r_[np.array([0,0,0]), -np.array(g)]
        # first acceleration is from gravity, dont know where the negative came from
        # this notation [:,0] is the first column
        # np.array is for case where g is inputed as python array, np.array works fine for g inputed as numpy array

    Mi = np.identity(4) # for the transfer from space axis to body axis, need the transf from {0} to the joint axis {i}
    Ai = np.zeros((6,n)) # body axes, need to store for second for loop
    AdTi = [[None]] * (n+1) # Adjoint of the transf_i_i-1, storing for second for loop
    AdTi[n] = ch3.adjoint_transf_matrix(ch3.transf_matrix_inverse(M_list[n])) 
        # second for loop wants the transf from last frame to tip, and first for loop doesnt provide it

    Fi = np.array(F_tip).copy() # copy by value, no link

    tau_list = np.zeros(n) # initialize torque return list

    for i in range(n): # 0 --> n-1
        Mi = np.dot(Mi, Mlist[i]) #previous iterations dotted with current link config (this ends up being current link referenced to {0})
        
        Ai[:,i] = np.dot(ch3.adjoint_transf_matrix(ch3.transf_matrix_inverse(Mi)), np.array(S_list)[:,i]) # Ai = Ad_(T_(i,i-1)) * Si
            # use adjoint_transf to convert a space frame screw axis Si, into screw axis of joint i in {i} Ai
            # Mi is T_sb, base relative to space; but to convert space axis to body axis--> Ab = T_bs*S_s (need to invert Mi)

        # Ti = e^(-A_i*theta)*M_i,i-1
        Ti = np.dot(ch3.se3_to_transf_matrix(ch3.vector6_to_se3(Ai[:, i]* -1*thetaList[i])),ch3.transf_matrix_inverse(Mlist[i]))
            # Ti is Mi,i-1 with the joints at the respective angles (M is home config, T is final config)
            # given Mi,i+1 in Mlist, need to invert it to fit in this equation
            # the [:,x] notation is editing the column, index of x
        AdTi[i] = ch3.adjoint_transf_matrix(Ti)

        # Vi = Ad_Ti,i-1(Vi-1) + A_i*theta_dot
        Va = np.dot(AdTi[i],Vi[:,i]) # twist of the previous link {i-1}, expressed in the current link {i} by the adjoint
        Vb = Ai[:,i]*d_thetaList[i] # joint rate theta_dot about the screw axis Ai
        Vi[:,i+1] = Va + Vb 
            # the [:,x] notation is editing the column, index of x

        # dVi = Ad_Ti,i-1(dVi-1) + ad_Vi(Ai)*theta_dot + A_i*theta_ddot
        Vda = np.dot(AdTi[i], Vdi[:,i]) # accleration of previous link {i-1}, expressed in current link {i} by adjoint
        Vdb = np.dot(adjoint_twist(Vi[:,i+1]),Ai[:,i]) * d_thetaList[i] # velocity product component (came from derivative of AdTi from Vi)
        Vdc = Ai[:,i]*dd_thetaList[i] # joint acceleration dd_theta around the screw axis Ai
        Vdi[:,i+1] = Vda+Vdb+Vdc

    for i in range(n-1,-1,-1):
        Fa = np.dot(np.array(AdTi[i+1]).T, Fi) # wrench applied to link i thru joint i+1
        Fb = np.dot(G_list[i],Vdi[:,i+1]) # wrench applied to link i thru joint i, G_b*dV_b
        Fc = np.dot(np.array(adjoint_twist(Vi[:,i+1])).T, np.dot(G_list[i], Vi[:,i+1])) # wrench applied to link i thru joint i (ad_Vb)^T*G_b*V_b
        Fi = Fa+Fb-Fc
        tau_list[i] = np.dot(np.array(Fi).T, Ai[:, i])

    return tau_list
    
# def mass_matrix(thetaList, MList, GList, SList):
#     # pg 296, do inverse dynamics and set g=0, \theta\dot = 0 , F_tip = 0
#     # need to init mass matrix like this, otherwise np.column_stack doenst work
#     n = len(thetaList)
#     mm = np.zeros(3)
#     d_thetaList = np.zeros(len(thetaList)) #\theta\dot=0

#     # init the double dot theta with all zeros, 1 in the first slot
#     dd_thetaList = np.zeros(len(thetaList))
#     dd_thetaList[0] = 1 

#     g = np.zeros(3) # g = 0
#     fTip = np.zeros(6) # fTip = 0

#     # each column of the mass matrix is found by setting 1 in the dd_thetaList=0 and inverse_dynamics 
#     while not np.all((dd_thetaList==0)):
#         torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList)

#         # moving the dd_thetaList for the next column (scooting the 1 right one space)
#         np.insert(dd_thetaList,0,0)
#         dd_thetaList = dd_thetaList[:-1]

#         # column stacking with existing mass matrix
#         mm = np.column_stack((mm, torque))

#     # remove that filler np.zeros(3) at the front of the mass matrix
#     return mm[1:]
def mass_matrix(thetaList, MList, GList, SList):

    n = len(thetaList)
    M = np.zeros((n, n))
    d_thetaList = np.zeros(n)
    g = np.zeros(3)
    fTip = np.zeros(6)
    for i in range (n):
        ddthetalist = np.zeros(n)
        ddthetalist[i] = 1
        M[:, i] = inverse_dynamics(thetaList, d_thetaList, ddthetalist, g, fTip, Mlist, Glist, Slist)
    
    return M

def vel_quadratic_force(thetaList, d_thetaList, M_list, G_list, S_list):
    # calling inverse_dynamics with no dd_theta and no F_tip
    dd_thetaList = np.zeros(len(thetaList))
    F_tip = np.zeros(6)
    g = np.zeros(3)

    torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list)
    return torque

def gravity_force(thetaList, g, M_list, G_list, S_list):
    # computes joint forces/torques needed to overcome gravity
    # basically, no movement in joints (d_theta=dd_theta=0), no outside forces (F_tip=0)
    d_thetaList = np.zeros(len(thetaList))
    dd_thetaList = d_thetaList 
    F_tip = np.zeros(6)

    torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list)
    return torque

def end_effector_force(thetaList, F_tip, M_list, G_list, S_list):
    # computes joint torques needed to counter the F_tip, doesn't include gravity

    d_thetaList = np.zeros(len(thetaList))
    dd_thetaList = d_thetaList 
    g = np.zeros(3)

    torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list)
    return torque

def forward_dynamics(thetaList, d_thetaList, tau_list, g, F_tip, M_list, G_list, S_list):
    mass = mass_matrix(thetaList, M_list, G_list, S_list)
    tau = np.array(tau_list)
    vel_quadratic = vel_quadratic_force(thetaList, d_thetaList, M_list, G_list, S_list)
    gravity = gravity_force(thetaList, g, M_list, G_list, S_list)
    tip = end_effector_force(thetaList, F_tip, M_list, G_list, S_list)

    a = np.dot(np.linalg.inv(mass), tau - vel_quadratic - gravity - tip)
    return a

def euler_step(thetaList, d_thetaList, dd_thetaList, dt):
    """Compute the joint angles and velocities at the next timestep using first order Euler integration
    :param thetalist: n-vector of joint variables
    :param dthetalist: n-vector of joint rates
    :param ddthetalist: n-vector of joint accelerations
    :param dt: The timestep delta t
    :return thetalistNext: Vector of joint variables after dt from first
                           order Euler integration
    :return dthetalistNext: Vector of joint rates after dt from first order
                            Euler integration
    Example Inputs (3 Link Robot):
        thetalist = np.array([0.1, 0.1, 0.1])
        dthetalist = np.array([0.1, 0.2, 0.3])
        ddthetalist = np.array([2, 1.5, 1])
        dt = 0.1
    Output:
        thetalistNext:
        array([ 0.11,  0.12,  0.13])
        dthetalistNext:
        array([ 0.3 ,  0.35,  0.4 ])
    """
    next_theta = thetaList + dt * np.array(d_thetaList)
    next_d_theta = d_thetaList + dt * np.array(dd_thetaList)

    return (next_theta, next_d_theta) # just like how x1 = x0 + v*dt 

def inverse_dynamic_trajectory(theta_matrix, d_theta_matrix, dd_theta_matrix, g, F_tip_matrix, M_list, G_list, S_list):
    """Calculates the joint forces/torques required to move the serial chain
    along the given trajectory using inverse dynamics
    :param thetamat: An N x n matrix of robot joint variables
    :param dthetamat: An N x n matrix of robot joint velocities
    :param ddthetamat: An N x n matrix of robot joint accelerations
    :param g: Gravity vector g
    :param Ftipmat: An N x 6 matrix of spatial forces applied by the end-
                    effector (If there are no tip forces the user should
                    input a zero and a zero matrix will be used)
    :param Mlist: List of link frames i relative to i-1 at the home position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :return: The N x n matrix of joint forces/torques for the specified
             trajectory, where each of the N rows is the vector of joint
             forces/torques at each time step"""

    # for each step, the theta,d_theta, dd_theta are calculated using some trajectory thingy in ch9
    # these are stored in the input matrices theta_matrix, d_theta_matrix, dd_theta_matrix (usually only 1D, and length is # of joints)
        # now it is 2D; main dimension is # of joints, but next dimension is # of steps in the trajectory
    
    # this is kinda stupid actually, because ideally you would calculate the tau as you create the new theta, d_theta, dd_theta matrices
        # you start looping to create these theta,d_theta, dd_theta (# of joints, # of steps) matrices, then you have to loop again in this function to access the 
        # matrices to create the torque matrix
        # im sure its good for readability though, and helps split chapters up

    # im pretty sure this is bad practice, reassigning a value in the arguments
    theta_matrix = np.array(theta_matrix).T 
    d_theta_matrix =  np.array(d_theta_matrix).T
    dd_theta_matrix = np.array(dd_theta_matrix).T
    F_tip_matrix = np.array(F_tip_matrix).T
        # i think this setup is for when the user inserts a python lists

    tau_matrix = np.array(theta_matrix).copy() # each step as n torques for n joints, the other dimension is the number of steps

    for i in range(np.array(theta_matrix).shape[1]):
        tau_matrix[:, i] = inverse_dynamics(theta_matrix[:, i], d_theta_matrix[:, i], \
                          dd_theta_matrix[:, i], g, F_tip_matrix[:, i], M_list, \
                          G_list, S_list)
    
    tau_matrix = np.array(tau_matrix).T
    return tau_matrix

def forward_dynamic_trajectory(thetaList, d_thetaList, tau_matrix, g, F_tip_matrix, M_list, G_list, S_list, dt, int_resolution):
    """Simulates the motion of a serial chain given an open-loop history of
    joint forces/torques
    :param thetalist: n-vector of initial joint variables
    :param dthetalist: n-vector of initial joint rates
    :param taumat: An N x n matrix of joint forces/torques, where each row is
                   the joint effort at any time step
    :param g: Gravity vector g
    :param Ftipmat: An N x 6 matrix of spatial forces applied by the end-
                    effector (If there are no tip forces the user should
                    input a zero and a zero matrix will be used)
    :param Mlist: List of link frames {i} relative to {i-1} at the home
                  position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :param dt: The timestep between consecutive joint forces/torques
    :param intRes: Integration resolution is the number of times integration
                   (Euler) takes places between each time step. Must be an
                   integer value greater than or equal to 1
    :return thetamat: The N x n matrix of robot joint angles resulting from
                      the specified joint forces/torques
    :return dthetamat: The N x n matrix of robot joint velocities
    This function calls a numerical integration procedure that uses
    ForwardDynamics."""

    tau_matrix = np.array(tau_matrix).T
    F_tip_matrix = np.array(F_tip_matrix).T
    theta_matrix = tau_matrix.copy().astype(np.float)
    theta_matrix[:, 0] = thetalist
    d_theta_matrix = tau_matrix.copy().astype(np.float)
    d_theta_matrix[:, 0] = d_thetaList
    for i in range(np.array(tau_matrix).shape[1] - 1): # use a set of torques (tau), loop thru each time step; the input is the set of torques at each timestep
        for j in range(int_resolution):
            dd_thetaList \
            = forward_dynamics(thetaList, d_thetaList, tau_matrix[:, i], g, \
                              F_tip_matrix[:, i], M_list, G_list, S_list) # finding the acceleration, based on current theta, dtheta
            thetaList,d_thetaList = euler_step(thetaList, d_thetaList, \
                                             dd_thetaList, 1.0 * dt / int_resolution) # updating theta, dtheta on new acceleration
            # repeat until you finish the number of times required in int_resolution
        
        theta_matrix[:, i + 1] = thetaList # store the final theta, d_theta into the theta_matrix, d_theta_matrix
        d_theta_matrix[:, i + 1] = d_thetaList
    
    theta_matrix = np.array(theta_matrix).T # dont know whats up with this tranpose
    d_theta_matrix = np.array(d_theta_matrix).T
    return theta_matrix, d_theta_matrix

thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
ddthetalist = np.array([2, 1.5, 1])
taulist = np.array([0.5, 0.6, 0.7])
g = np.array([0, 0, -9.8])
Ftip = np.array([1, 1, 1, 1, 1, 1])
M01 = np.array([[1, 0, 0,        0],
                [0, 1, 0,        0],
                [0, 0, 1, 0.089159],
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28],
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0],
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395],
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0],
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225],
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                    [0, 1, 0, -0.089, 0,     0],
                    [0, 1, 0, -0.089, 0, 0.425]]).T

a = inverse_dynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
print(a)

b = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
print(b)

mm = mass_matrix(thetalist, Mlist, Glist, Slist)
print(mm)

c = forward_dynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
print(c)
