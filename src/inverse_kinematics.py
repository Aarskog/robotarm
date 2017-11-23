import numpy as np
import math

from scipy.optimize import minimize
from numpy import linalg as LA

def forward_kinematics(q):
    d1 = 0.0796
    a2 = 0.1347
    a3 = 0.0712
    d5 = 0.0918

    c1   = np.cos(q[0])
    c2   = np.cos(q[1])
    c23  = np.cos(q[1] + q[2])
    c234 = np.cos(q[1] + q[2] + q[3])
    c5   = np.cos(q[4])

    s1   = np.sin(q[0])
    s2   = np.sin(q[1])
    s5   = np.sin(q[4])
    s23  = np.sin(q[1] + q[2])
    s234 = np.sin(q[1] + q[2] + q[3])

    lon = a3*s23+a2*s2+d5*s234

    T5 = np.matrix([
    [c1*c5 - c234*s1*s5,-c1*s5-c234*c5*s1,s234*s1,s1*lon],
    [c5*s1+c234*c1*s5, c234*c1*c5-s1*s5,-s234*c1,-c1*lon],
    [s234*s5,s234*c5,c234, d1 + a3*c23+a2*c2+d5*c234],
    [0,0,0,1]
    ])


    #T5 = np.matrix([
    #[c1*c234*c5 + s1*s5, -c1*c234*s5 + s1*c5, -c1*s234, c1*lon],
    #[c1*c234*c5 - s1*s5, -s1*c234*s5 - c1*c5, -s1*s234, s1*lon],
    #[-s234*c5, s234*s5, -c234, -d1 - a2*s2-a3*s23 - d5*c234],
    #[0,0,0,1]
    #])
    return T5

def direct_kinematics(q):
    T5 = forward_kinematics(q)
    p5 = T5[0:3,3]
    eul = get_euler_angles_zyx(T5)
    return np.concatenate((p5,eul),axis=0)

def norm_of_error(q,xd):
    err = direct_kinematics(q)-xd
    return LA.norm(err)

def get_euler_angles_zyx(T5):
    #Get values form rotation matrix in T5
    r21 = T5[1,0]
    r11 = T5[0,0]
    r31 = T5[2,0]
    r32 = T5[2,1]
    r33 = T5[2,2]

    thetaz = math.atan2(r21,r11)
    thetay = math.atan2(-r31,np.sqrt(r32*r32+r33*r33))
    thetax = math.atan2(r32,r33)

    return np.matrix([[thetaz],[thetay],[thetax]])

def inverse_kinematics_optimization(xd,method):
    q0 = np.random.random(5)
    #q0 = np.matrix([[-1.5708],[0.42],[0],[-1.9849],[1.5708]])
    optimized = minimize(norm_of_error,q0,args=(xd),method=method)

    restarts = 0
    error_tolerance = 0.01
    maxiter = 4
    best_solution = optimized

    while optimized.fun > error_tolerance and maxiter>0:
        q0 = (np.random.random(5)*2-1)*np.pi

        optimized = minimize(norm_of_error,q0,args=(xd,))#,method=method)
        restarts += 1
        maxiter

        if optimized.fun < best_solution.fun:
            best_solution = optimized
            print q0
        maxiter -= 1

    np.set_printoptions(suppress=True)
    print optimized
    print direct_kinematics(optimized.x)
    print restarts
    qopt = optimized.x
    i=0
    for item in qopt:
        qopt[i] = ( item + np.pi) % (2 * np.pi ) - np.pi
        i = i + 1

    return qopt
