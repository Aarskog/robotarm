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


#--- State variables
 # Position
q = np.matrix([0.0,0.0,0.0,0.0,0.0],dtype=float)

 # velocity
q_dot = np.matrix([0,0,0,0,0],dtype=float)


#---Position and velocity from gazebo
def callback(data):
    i=0
    while i<q.shape[1]:
        q[0,i]=data.position[i]
        q_dot[0,i] = data.velocity[i]
        i+=1
#--



#Define a RRBot joint positions publisher for joint controllers.
def five_dof_robotarm_joint_positions_publisher():

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

    rate = rospy.Rate(100) #100 Hz

    # Desired joint positions
    qd = np.matrix([0.0,0.1,0.1,0.0,0.0])

    #
    kp1 = 1
    kp2 = 1
    kp3 = 1
    kp4 = 1
    kp5 = 1
    Kp = np.matrix([[kp1,0,0,0,0],[0,kp2,0,0,0],[0,0,kp3,0,0],[0,0,0,kp4,0],[0,0,0,0,kp5]],dtype=float)

    kd1 = 1
    kd2 = 1
    kd3 = 1
    kd4 = 1
    kd5 = 1
    Kd = np.matrix([[kd1,0,0,0,0],[0,kd1,0,0,0],[0,0,kd1,0,0],[0,0,0,kd1,0],[0,0,0,0,kd1]],dtype=float)


	#While loop to have joints follow a certain position, while rospy is not shutdown.
    i = float(0)
    while not rospy.is_shutdown():
        q_tilde = qd-q
        u = np.matmul(Kp,np.transpose(q_tilde))-np.matmul(Kd,np.transpose(q_dot))

        print u
        #rospy.loginfo(i/100)
		#Publish the same sine movement to each joint.
        pub1.publish(u[0,0])
        pub2.publish(u[1,0])
        pub3.publish(u[2,0])
        pub4.publish(u[3,0])
        pub5.publish(u[4,0])



        i = i+1 #increment i
        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':


    try: five_dof_robotarm_joint_positions_publisher()
    except rospy.ROSInterruptException: pass
