#!/usr/bin/env python

# Source
# https://github.com/JoshMarino/gazebo_and_ros_control/tree/master/rrrbot_files/src

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs,pi

#Define a RRBot joint positions publisher for joint controllers.
def five_dof_robotarm_joint_positions_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
    pub1 = rospy.Publisher('/five_dof_robotarm/base_to_turntable_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/five_dof_robotarm/second_joint_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/five_dof_robotarm/third_joint_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/five_dof_robotarm/fourth_joint_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(1) #100 Hz


    jointPos = [0,0,0,0,0]

	#While loop to have joints follow a certain position, while rospy is not shutdown.
    i = float(0)
    while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
        sine_movement = 2*sin(i/100.)
        if (i/100)>(2*pi):
            i = float(0)
        jointPos[0] = 0.6
        jointPos[1] = 0.9
        jointPos[2] = 0.4
        jointPos[3] = 0.44

        #rospy.loginfo(i/100)
		#Publish the same sine movement to each joint.
        pub1.publish(jointPos[0])
        pub2.publish(jointPos[1])
        pub3.publish(jointPos[2])
        pub4.publish(jointPos[3])


        i = i+1 #increment i
        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: five_dof_robotarm_joint_positions_publisher()
    except rospy.ROSInterruptException: pass
