#!/usr/bin/env python

# Source
# https://github.com/JoshMarino/gazebo_and_ros_control/tree/master/rrrbot_files/src

import rospy
import math
import numpy as np
import inverse_kinematics as ik

from scipy.interpolate import CubicSpline

from numpy.linalg import inv

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from math import sin,cos,atan2,sqrt,fabs,pi

debug = False

 # Position
q = np.matrix([0.0,0.0,0.0,0.0,0.0],dtype=float)

 # velocity
q_dot = np.matrix([0,0,0,0,0],dtype=float)

#Link Masses
lm = {'m1':0.126+0.05,'m2':0.01+0.03+0.126,'m3':0.126+0.01,'m4':0.126+0.01,'m5':0.126+0.01*2}

#Link Length
ll = {'l1':0.0506+0.01,'l2':0.0206+0.0635+0.0506,'l3':0.0206+0.0506,'l4':0.0206+0.0506,'l5':0.0506/2+0.01}

#Gravity
g = -9.81

#Timestamp

timestamp = float(0)

#Position and velocity from Gazebo
def callback(data):
    global timestamp

    q[0,0] = data.position[0]
    q[0,1] = data.position[2]
    q[0,2] = data.position[3]
    q[0,3] = data.position[1]
    q[0,4] = data.position[4]

    q_dot[0,0] = data.velocity[0]
    q_dot[0,1] = data.velocity[2]
    q_dot[0,2] = data.velocity[3]
    q_dot[0,3] = data.velocity[1]
    q_dot[0,4] = data.velocity[4]

    timestamp = data.header.stamp.secs + float(float(data.header.stamp.nsecs)/float(10**9))

def create_spline(qd,spline_step):


    qd_set     = [[],[],[],[],[]]
    data_space = [[],[],[],[],[]]


    #Transform data such that we get the joint postitions in each row
    qd = qd.transpose()

    i = 0
    for qdi in qd:
        x = np.arange(0,qdi.shape[1])
        y = np.squeeze(np.asarray(qdi))

        cs = CubicSpline(x,y)

        data_space[i] = np.arange(0,qdi.shape[1]-spline_step,spline_step)

        qd_set[i]=cs(data_space[i])

        i = i + 1


    #plt.plot(data_space[1],qd_set[1])
    #plt.show()
    return qd_set

def gravity():
    global q,q_dot,g,lm,ll,timestamp

    c234 = sin(q[0,1]+q[0,2]+q[0,3])
    c23 = sin(q[0,1]+q[0,2])
    c2 = sin(q[0,1])


    l23 = lm['m3']*g*(ll['l2']*c2 + ll['l3']/2*c23)
    l24 = lm['m4']*g*(ll['l2']*c2 + ll['l3']*c23 + ll['l4']/2*c234)
    l25 = lm['m5']*g*(ll['l2']*c2 + ll['l3']*c23 + (ll['l4']+ll['l5']/2)*c234)

    l34 = lm['m4']*g*(ll['l3']*c23 + ll['l4']/2*c234)
    l35 = lm['m5']*g*(ll['l3']*c23 + (ll['l4']+ll['l5']/2)*c234)

    l45 = lm['m5']*g*(ll['l4']+ll['l5']/2)*c234

    g_1 = 0
    g_2 = lm['m2']*g*ll['l2']/2*c2  + l23 + l24 +l25
    g_3 = lm['m3']*g*ll['l3']/2*c23 + l34 + l35
    g_4 = lm['m4']*g*ll['l4']/2*c234 +l45
    g_5 = 0
    return np.matrix([[g_1],[g_2],[g_3],[g_4],[g_5]])

#Get the next set point from a set of spline points
def get_qd(qd_set,q,iterations,qd,coa):

    for i in range(0,len(iterations)):
        if (abs(qd[i]-q[0,i]) < coa) and ((iterations[i]+1)<(len(qd_set[i]))):
            iterations[i] += 1
            qd[i] = qd_set[i][iterations[i]]

    return qd

#Define a RRBot joint positions publisher for joint controllers.
def five_dof_robotarm_joint_positions_publisher(Kp,Kd,qd,spline_step,coa):
    global q,q_dot,g,lm,ll,timestamp
	#Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
    pub1 = rospy.Publisher('/five_dof_robotarm/base_to_turntable_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/five_dof_robotarm/second_joint_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/five_dof_robotarm/third_joint_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/five_dof_robotarm/fourth_joint_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/five_dof_robotarm/fifth_joint_controller/command', Float64, queue_size=10)

    #Get the joint positions and velocities
    sub = rospy.Subscriber('/five_dof_robotarm/joint_states', JointState, callback)

    update_rate = 1000
    rate = rospy.Rate(update_rate) #100 Hz

    #qd_set = create_spline(np.matrix(qd),spline_step)
    qd = np.matrix(qd)
    iterations = [0,0,0,0,0]

    qda = [0.0,0.0,0.0,0.0,0.0]

	#While loop to have joints follow a certain position, while rospy is not shutdown.
    i = float(0)
    datafile = open('/home/magnaars/catkin_ws/src/five_dof_robotarm/src/datafile.txt','w')
    while not rospy.is_shutdown():
        #qda = get_qd(qd_set,q,iterations,qda,coa)
        #qd = np.matrix(qda)
        q_tilde = qd-q
        gravity_gain = gravity()

        #u = Kp*q_tilde - Kd*q_dot + g(q)
        u = np.matmul(Kp,np.transpose(q_tilde))-np.matmul(Kd,np.transpose(q_dot)) + gravity_gain

        #Send the wanted torques to Gazebo
        pub1.publish(u[0,0])
        pub2.publish(u[1,0])
        pub3.publish(u[2,0])
        pub4.publish(u[3,0])
        pub5.publish(u[4,0])

        #Print every second instead of every iteration
        if (i%update_rate==0 or i==0) and debug:
            print "\n\n----------------------------------------------------------------"
            print "iterations = ",iterations
            print "qd = \n",np.transpose(qd)
            print "q = \n",np.transpose(q)
            print "q_tilde = \n",np.transpose(q_tilde)
            print "u = \n",u
            print "g = \n",gravity_gain
            print "t = \n",timestamp


        tstamp = np.matrix([timestamp,0,0,0,0])

        #Save robot data to file. Used for plotting
        np.savetxt(datafile,qd)
        np.savetxt(datafile,q)
        np.savetxt(datafile,q_tilde)
        np.savetxt(datafile,np.transpose(u))
        np.savetxt(datafile,np.transpose(gravity_gain))
        np.savetxt(datafile,tstamp)


        i = i+1
        rate.sleep()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':

    spline_step = 0.1

    #Circle of acceptrance
    coa = 0.03

    # Kp gain
    kp1 = 1.4
    kp2 = 6
    kp3 = kp2*1.1
    kp4 = kp2*1.5
    kp5 = 10
    Kp = np.matrix([
    [kp1,0,0,0,0],
    [0,kp2,0,0,0],
    [0,0,kp3,0,0],
    [0,0,0,kp4,0],
    [0,0,0,0,kp5]],
    dtype=float)

    #Kd gain
    kd1 = 1
    kd2 = 0.9
    kd3 = kd2
    kd4 = kd2
    kd5 = 1
    Kd = np.matrix([
    [kd1,0,0,0,0],
    [0,kd2,0,0,0],
    [0,0,kd3,0,0],
    [0,0,0,kd4,0],
    [0,0,0,0,kd5]],
    dtype=float)

    #Start pos
    q0 = [0,0,0,0,0]


    #qOri = [1.5708,-0.6624,0.5362,1.6970,-1.5708]
    #qtest = [-3.1416,-0.8612,0.0,-0.7074,3.1416]

    # Desired joint positions
    #qd = [q0,qOri]#,[3.13,1.5,1.2,-1,6]]
    #qd = qtest

    d1 = 0.0796
    a2 = 0.1347
    a3 = 0.0712
    d5 = 0.0918

    pd = np.matrix([[0],[0],[d1+a2+a3]])
    od = np.matrix([[0],[np.pi/2],[0]])
    xd = np.concatenate((pd,od),axis=0)

    #qopt = ik.inverse_kinematics_optimization(xd,'BFGS')
    #qd = qopt
    #print qopt
    rospy.init_node('joint_positions_node', anonymous=True)
    update_rate = 300
    rate = rospy.Rate(update_rate)

    while not rospy.is_shutdown():
        rate.sleep()
    #try: five_dof_robotarm_joint_positions_publisher(Kp,Kd,qd,spline_step,coa)
    #except rospy.ROSInterruptException: pass
