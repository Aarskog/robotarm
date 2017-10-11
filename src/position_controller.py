#!/usr/bin/env python

# Source
# https://github.com/JoshMarino/gazebo_and_ros_control/tree/master/rrrbot_files/src

import rospy
import math
import numpy as np

from numpy.linalg import inv

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from math import sin,cos,atan2,sqrt,fabs,pi

debug = True

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

#---Position and velocity from Gazebo
def callback(data):

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

#--

def gravity():
    global q,q_dot,g,lm,ll

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

#Define a RRBot joint positions publisher for joint controllers.
def five_dof_robotarm_joint_positions_publisher():
    global q,q_dot,g,lm,ll
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


    # Kp gain
    kp1 = 2
    kp2 = 5
    kp3 = kp2/2
    kp4 = kp2/3
    kp5 = 2
    Kp = np.matrix([
    [kp1,0,0,0,0],
    [0,kp2,0,0,0],
    [0,0,kp3,0,0],
    [0,0,0,kp4,0],
    [0,0,0,0,kp5]],
    dtype=float)

    #Kd gain
    kd1 = 2
    kd2 = 0.5
    kd3 = kd2
    kd4 = kd2
    kd5 = 2
    Kd = np.matrix([
    [kd1,0,0,0,0],
    [0,kd2,0,0,0],
    [0,0,kd3,0,0],
    [0,0,0,kd4,0],
    [0,0,0,0,kd5]],
    dtype=float)

    # Desired joint positions
    qd = np.matrix([3,0.6,-0.3,-0.3,3])
    #qd = qd*0
	#While loop to have joints follow a certain position, while rospy is not shutdown.
    i = float(0)
    while not rospy.is_shutdown():
        q_tilde = qd-q
        gravity_gain = gravity()


        u = np.matmul(Kp,np.transpose(q_tilde))-np.matmul(Kd,np.transpose(q_dot)) + gravity_gain
        #print q_tilde
        #rospy.loginfo(i/100)
		#Publish the same sine movement to each joint.
        pub1.publish(u[0,0])
        pub2.publish(u[1,0])
        pub3.publish(u[2,0])
        pub4.publish(u[3,0])
        pub5.publish(u[4,0])

        #Print every second instead of every iteration
        if (i%update_rate==0 or i==0) and debug:
            print "\n\n--------------------------Debug-----------------------------------"
            print "q_tilde = \n",np.transpose(q_tilde)
            print "u = \n",u
            print "g = \n",gravity_gain
            print "q = \n",np.transpose(q)


        i = i+1 #increment i
        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':


    try: five_dof_robotarm_joint_positions_publisher()
    except rospy.ROSInterruptException: pass
    signal.signal(signal.SIGINT, shutdown)
