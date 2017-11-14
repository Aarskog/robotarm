import numpy as np
import matplotlib.pyplot as plt
import math

#-----

from scipy.interpolate import CubicSpline


def create_spline(qd):
    step = 0.1

    qd_set     = [[],[],[],[],[]]
    data_space = [[],[],[],[],[]]


    #Transform data
    qd = qd.transpose()

    i = 0
    for qdi in qd:
        x = np.arange(0,qdi.shape[1])
        y = np.squeeze(np.asarray(qdi))

        cs = CubicSpline(x,y)

        data_space[i] = np.arange(0,qdi.shape[1]-1,step)

        qd_set[i]=cs(data_space[i])

        i = i + 1


    #plt.plot(data_space[1],qd_set[1])
    #plt.show()
    return qd_set

def readDataFromfile(filename):

    datafile = open(filename,'r')
    #Load every line except the last one since it may be incomplete, because
    # ctrl-c is used and cutting in the middle of writing to file is possible.
    datas = np.genfromtxt(datafile,skip_footer=1)

    data_from_robot = {'qd':[],'q':[],'q_tilde':[],'u':[],'g':[],'t':[]}

    for i in range(0,datas.shape[0]-5,6):
        data_from_robot['qd'].append(datas[i])
        data_from_robot['q'].append(datas[i+1])
        data_from_robot['q_tilde'].append(datas[i+2])
        data_from_robot['u'].append(datas[i+3])
        data_from_robot['g'].append(datas[i+4])
        data_from_robot['t'].append(datas[i+5])


    datafile.close()
    return data_from_robot

def plotJointAngles(dtf,radians,degrees,autos):

    t   = [item[0] for item in dtf['t']]

    q1 = [item[0] for item in dtf['q']]
    q2 = [item[1] for item in dtf['q']]
    q3 = [item[2] for item in dtf['q']]
    q4 = [item[3] for item in dtf['q']]
    q5 = [item[4] for item in dtf['q']]

    qd1 = [item[0] for item in dtf['qd']]
    qd2 = [item[1] for item in dtf['qd']]
    qd3 = [item[2] for item in dtf['qd']]
    qd4 = [item[3] for item in dtf['qd']]
    qd5 = [item[4] for item in dtf['qd']]



    if radians:
        plt.figure(1)

        plt.subplot(511)
        plt.plot(t,q1,'r',t,qd1)
        plt.title('Measured joint angles and desired joint angles')
        plt.xlabel('time(seconds)')
        plt.ylabel('q_1 (rad)')
        plt.ylim(ylimitLO,ylimitUP)
        plt.autoscale(autos)

        plt.subplot(512)
        plt.plot(t,q2,'r',t,qd2)
        plt.xlabel('time(seconds)')
        plt.ylabel('q_2 (rad)')
        plt.ylim(ylimitLO,ylimitUP)
        plt.autoscale(autos)

        plt.subplot(513)
        plt.plot(t,q3,'r',t,qd3)
        plt.xlabel('time(seconds)')
        plt.ylabel('q_3 (rad)')
        plt.ylim(ylimitLO,ylimitUP)
        plt.autoscale(autos)

        plt.subplot(514)
        plt.plot(t,q4,'r',t,qd4)
        plt.xlabel('time(seconds)')
        plt.ylabel('q_4 (rad)')
        plt.ylim(ylimitLO,ylimitUP)
        plt.autoscale(autos)

        plt.subplot(515)
        plt.plot(t,q5,'r',t,qd5)
        plt.xlabel('time(seconds)')
        plt.ylabel('q_5 (rad)')
        plt.ylim(ylimitLO,ylimitUP)
        plt.autoscale(autos)

        plt.show()

    if degrees:
        plt.figure(2)

        plt.subplot(511)
        plt.plot(t,np.degrees(q1),'r',t,np.degrees(qd1))
        plt.title('Measured joint angles and desired joint angles')
        plt.xlabel('time(seconds)')
        plt.ylabel('q_1 (deg)')
        plt.ylim(ylimitLO,ylimitHI)
        plt.autoscale(autos)

        plt.subplot(512)
        plt.plot(t,np.degrees(q2),'r',t,np.degrees(qd2))
        plt.xlabel('time(seconds)')
        plt.ylabel('q_2 (deg)')
        plt.ylim(ylimitLO,ylimitHI)
        plt.autoscale(autos)

        plt.subplot(513)
        plt.plot(t,np.degrees(q3),'r',t,np.degrees(qd3))
        plt.xlabel('time(seconds)')
        plt.ylabel('q_3 (deg)')
        plt.ylim(ylimitLO,ylimitHI)
        plt.autoscale(autos)

        plt.subplot(514)
        plt.plot(t,np.degrees(q4),'r',t,np.degrees(qd4))
        plt.xlabel('time(seconds)')
        plt.ylabel('q_4 (deg)')
        plt.ylim(ylimitLO,ylimitHI)
        plt.autoscale(autos)

        plt.subplot(515)
        plt.plot(t,np.degrees(q5),'r',t,np.degrees(qd5))
        plt.xlabel('time(seconds)')
        plt.ylabel('q_5 (deg)')
        plt.ylim(ylimitLO,ylimitHI)
        plt.autoscale(autos)

        plt.show()


if __name__ == '__main__':
    filename = '/home/magnaars/catkin_ws/src/five_dof_robotarm/src/datafile.txt'

    dtf = readDataFromfile(filename)

    #Upper and lower limits of y-axis
    ylimitHI = float('nan')
    ylimitLO = float('nan')

    #Choose to plot in rad or deg
    radians = False
    degrees = True

    #autoscale
    autos = True

    plotJointAngles(dtf,radians,degrees,autos)
