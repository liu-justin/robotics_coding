import chapter3 as ch3 
import chapter4 as ch4 
import chapter5 as ch5
import chapter6 as ch6

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

def inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList):
    forces = np.array(fTip)
    torques = []
    transfs = []
    twists = [np.array([0,0,0,0,0,0])] # first twist shouldn't be moving if arm is stationary, if attached to a wheeled platform then yeah change it
    twistDots = [np.append(np.zeros(3),-1*g)]

    # have to edit both for loops for i=1..n, i=n...1 (cut some lists up) hard

    for (theta, dtheta, ddtheta, M, A) in zip(thetaList, d_thetaList, dd_thetaList, MList, np.transpose(SList)):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        print(A)
        T = ch3.screwtheta_to_transf_matrix(A,theta) @ M
        V = np.dot(ch3.adjoint_transf_matrix(T),twists[-1]) + A*dtheta
        Vdot = np.dot(ch3.adjoint_transf_matrix(T), twistDots[-1]) + np.dot(adjoint_twist(V),A)*dtheta + A*ddtheta 

        transfs.append(T)
        twists.append(V)
        twistDots.append(Vdot)

    # twists and twistDots contain the zeroth twist and twistDot, but this zip will stop at the smallest list end, so it fine
    for (twist, twistDot, G, A) in zip(twists[::-1], twistDots[::-1], GList[::-1], np.transpose(SList)[::-1]):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        # something wrong here
        force = np.dot(ch3.adjoint_transf_matrix(T),forces[0]) + np.dot(G,twistDot) - np.dot(adjoint_twist(V),(np.dot(G,twist)))
        tau = np.dot(force,A)

        np.column_stack((forces,force))
        torques.insert(0,tau)

    return torques

def inverse_dynamics_closedForm(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list):
    A = np.diag(S_list)
    G = np.diag(G_list)

    V_0 = np.zeros(3)
    V_base = np.dot(ch3.adjoint_transf_matrix(M_list[0]), V_0) # its not this, what is T_10

    W = np.zeros((4,4))
    for T in M_list():
        np.column_stack((W,ch3.adjoint_transf_matrix(T)))

    W = np.diag(W, -1)

    ad_transf = np.array([])
    for (A, d_theta) in zip(S_list, d_thetaList):
        np.append(ad_transf, adjoint_twist(A*d_theta))

    ad_transf = np.diag(ad_transf)

def mass_matrix(thetaList, MList, GList, SList):
    # pg 296, do inverse dynamics and set g=0, \theta\dot = 0 , F_tip = 0
    # need to init mass matrix like this, otherwise np.column_stack doenst work
    mm = np.zeros(3)
    d_thetaList = np.zeros(len(thetaList)) #\theta\dot=0

    # init the double dot theta with all zeros, 1 in the first slot
    dd_thetaList = np.zeros(len(thetaList))
    dd_thetaList[0] = 1 

    g = np.zeros(3) # g = 0
    fTip = np.zeros(3) # fTip = 0

    # each column of the mass matrix is found by setting 1 in the dd_thetaList=0 and inverse_dynamics 
    while not np.all((dd_thetaList==0)):
        torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList)

        # moving the dd_thetaList for the next column (scooting the 1 right one space)
        np.insert(dd_thetaList,0,0)
        dd_thetaList = dd_thetaList[:-1]

        # column stacking with existing mass matrix
        mm = np.column_stack((mm, torque))

    # remove that filler np.zeros(3) at the front of the mass matrix
    return mm[1:]

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

def end_effector_force(thetaList, F_tip, M_list, G_list, S_list):
    # computes joint torques needed to counter the F_tip, doesn't include gravity

    d_thetaList = np.zeros(len(thetaList))
    dd_thetaList = d_thetaList 
    g = np.zeros(3)

    torque = inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list)

def forward_dynamics(thetaList, d_thetaList, tau_list, g, F_tip, M_list, G_list, S_list):
    peepeepoopoo = 1212


thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
ddthetalist = np.array([2, 1.5, 1])
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