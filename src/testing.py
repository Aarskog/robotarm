import inverse_kinematics as ik
import numpy as np

if __name__ == '__main__':

    d1 = 0.0796
    a2 = 0.1347
    a3 = 0.0712
    d5 = 0.0918

    pd = np.matrix([[0],[0],[d1+a2+a3]])
    od = np.matrix([[0],[np.pi/2],[0]])
    xd = np.concatenate((pd,od),axis=0)

    #Methods
    #Powell
    #Nelder-Mead
    #BFGS
    qopt = ik.inverse_kinematics_optimization(xd,'BFGS')
    #q0 = np.matrix([[0],[-0.4204],[0],[1.9849],[0]])
    #print ik.norm_of_error(q0,xd)
    #print ik.direct_kinematics(q0)
    print xd
