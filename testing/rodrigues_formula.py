import numpy as np 
import math

def skew_symmetric(x):
    return np.array([[    0, -x[2],  x[1]],
                     [ x[2],     0, -x[0]],
                     [-x[1],  x[0],     0]])

def rodrigues(omega, theta):
    first = np.identity(3)
    print(f"this is first: {first}")
    skewed_omega = skew_symmetric(omega)
    second = math.sin(theta)*skewed_omega
    print(f"this is second: {second}")
    third = (1-math.cos(theta))*np.dot(skewed_omega,skewed_omega)
    print(f"this is third: {third}")

    return first+second+third

z_axis = np.array([1/math.sqrt(2),1/math.sqrt(2),0])
final_rotation = rodrigues(z_axis, np.pi/6)
print(np.round(final_rotation,5))

A = np.array([[-2, 1],[0,-1]])
print(np.dot(A,np.dot(A,A)))