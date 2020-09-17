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

def inverse_dynamics(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList):
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

def inverse_dynamics_closedForm(thetaList, d_thetaList, dd_thetaList, g, F_tip, M_list, G_list, S_list):
    n = len(thetaList)

    # initializing the twist, and twist dot
    Vi = np.zeros((6,n+1)) # if there are 3 joints, then there are 4 twists to find
    Vdi = np.zeros((6,n+1))
    # this notation [:,0] is the first column
    Vdi[:,0] = np.r_[np.array([0,0,0]), -np.array(g)] # first acceleration is from gravity, although dont know why its on the first d_twist

    Mi = np.identity(4) # for the transfer from space axis to body axis, need the transf from {0} to the joint axis {i}
    Ai = np.zeros((6,n)) # this code from github and textbook seems off
    AdTi = [[None]] * (n+1)
    AdTi[n] = ch3.adjoint_transf_matrix(ch3.transf_matrix_inverse(M_list[n])) # no idea what this is

    Fi = np.array(F_tip).copy() # copy by value, no link

    tau_list = np.zeros(n) # initialize tau return list

    for i in range(n): # 0 --> n-1
        Mi = np.dot(Mi, Mlist[i]) #previous iterations dotted with current link config (this ends up being current link referenced to {0})
        
        # use adjoint_transf to convert a space frame screw axis Si, into screw axis of joint i in {i} Ai
        # this is the reason Mi is counted, transf from current link i to base {0} is not in the inputs, need to keep a tracker on it
        # Mi is T_sb, base relative to space; to convert space axis to body axis--> Ab = T_bs*S_s (need to invert Mi)
        Ai[:,i] = np.dot(ch3.adjoint_transf_matrix(ch3.transf_matrix_inverse(Mi)), np.array(S_list)[:,i]) # Ai = Ad_(T_(i,i-1)) * Si

        # Ti = e^(-A_i*theta)*M_i,i-1 --> given Mi,i+1 in Mlist, need to invert it
        Ti = np.dot(ch3.se3_to_transf_matrix(ch3.vector6_to_se3(Ai[:, i]* -1*thetaList[i])),ch3.transf_matrix_inverse(Mlist[i]))
        AdTi[i] = ch3.adjoint_transf_matrix(Ti)

        # Vi = Ad_Ti,i-1(Vi-1) + A_i*theta_dot
        # the [:,x] notation is editing the column, index of x
        Vi[:,i+1] = np.dot(AdTi[i],Vi[:,i]) + Ai[:,i]*d_thetaList[i]

        # dVi = Ad_Ti,i-1(dVi-1) + ad_Vi(Ai)*theta_dot + A_i*theta_ddot
        a = np.dot(AdTi[i], Vdi[:,i])
        b = np.dot(adjoint_twist(Vi[:,i+1]),Ai[:,i]) * d_thetaList[i]
        c = Ai[:,i]*dd_thetaList[i]
        Vdi[:,i+1] = a+b+c

        # Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) \
        #                + Ai[:, i] * dd_thetaList[i] \
        #                + np.dot(adjoint_twist(Vi[:, i + 1]), Ai[:, i]) * d_thetaList[i]
        #Vdi[:,i+1] = np.dot(AdTi[i], Vdi[:,i]) + np.dot(adjoint_twist(Vi[:,i+1]),Ai[:,i]) * d_thetaList[i] + Ai[:,i]*dd_thetaList[i]
    print(Vdi)


    for i in range(n-1,-1,-1):
        Fa = np.dot(np.array(AdTi[i+1]).T, Fi)
        Fb = np.dot(G_list[i],Vdi[:,i+1])
        Fc = np.dot(np.array(adjoint_twist(Vi[:,i+1])).T, np.dot(G_list[i], Vi[:,i+1]))
        Fi = Fa+Fb-Fc
        # Fi = np.dot(np.array(AdTi[i+1]).T, Fi) + np.dot(G_list[i],Vdi[:,i+1]) - np.dot(np.array(adjoint_twist(Vi[:,i])).T, np.dot(G_list[i], Vi[:,i+1]))
        tau_list[i] = np.dot(np.array(Fi).T, Ai[:, i])

    return tau_list



    # A = np.diag(S_list)
    # G = np.diag(G_list)

    # V_0 = np.zeros(3)
    # V_base = np.dot(ch3.adjoint_transf_matrix(M_list[0]), V_0) # its not this, what is T_10

    # W = np.zeros((4,4))
    # for T in M_list():
    #     np.column_stack((W,ch3.adjoint_transf_matrix(T)))

    # W = np.diag(W, -1)

    # ad_transf = np.array([])
    # for (A, d_theta) in zip(S_list, d_thetaList):
    #     np.append(ad_transf, adjoint_twist(A*d_theta))

    # ad_transf = np.diag(ad_transf)

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

a = inverse_dynamics_closedForm(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
print(a)

b = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
print(b)