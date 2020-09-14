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

def inverse_dynamics_hard(thetaList, d_thetaList, dd_thetaList, g, fTip, MList, GList, SList):
    forces = [np.append(np.zeros(3), fTip)]
    torques = []
    transfs = []
    twists = [np.array([0,0,0,0,0,0])] # first twist shouldn't be moving if arm is stationary, if attached to a wheeled platform then yeah change it
    twistDots = [np.append(np.zeros(3),-1*g)]

    # have to edit both for loops for i=1..n, i=n...1 (cut some lists up) hard

    for (theta, dtheta, ddtheta, M, A) in zip(thetaList, d_thetaList, dd_thetaList, MList, SList):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        T = ch3.screwtheta_to_transf_matrix(A,theta) @ M
        V = np.dot(ch3.adjoint_transf_matrix(T),twists[-1]) + A*dtheta
        Vdot = np.dot(ch3.adjoint_transf_matrix(T), twistDots[-1]) + np.dot(adjoint_twist(V),A)*dtheta + A*ddtheta 

        transfs.append(T)
        twists.append(V)
        twistDots.append(Vdot)

    # twists and twistDots contain the zeroth twist and twistDot, but this zip will stop at the smallest list end, so it fine
    for (twist, twistDot, G, A) in zip(twists[::-1], twistDots[::-1], GList[::-1], SList[::-1]):
        #first need to find the transf matrix T_(i+1,i) using M,A,thetaand matrix exponentials
        force = np.dot(ch3.adjoint_transf_matrix(T),forces[0]) + np.dot(G,twistDot) - np.dot(adjoint_twist(V),(np.dot(G,twist)))
        tau = np.transpose(force)*A

        forces.insert(0,force)
        torques.insert(0,tau)

    return torques

def mass_matrix(thetaList, MList, GList, SList):

twist = np.array([1,2,3,4,5,6])
print(adjoint_twist(twist))