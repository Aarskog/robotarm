
import inverse_kinematics as ik
import numpy as np
import openravepy

#moveit
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import getRobotRBT

#-----------




if __name__ == '__main__':

    d1 = 0.0796
    a2 = 0.1347
    a3 = 0.0712
    d5 = 0.0918

    pd = np.matrix([[0],[0],[d1+a2+a3-0.1]])
    od = np.matrix([[0],[np.pi/2],[0]])
    xd = np.concatenate((pd,od),axis=0)

    #Methods
    #Powell
    #Nelder-Mead
    #BFGS
    qopt = ik.inverse_kinematics_optimization(xd,'BFGS')

    print qopt
    print ik.direct_kinematics(qopt)



    #remember to start roscore
    if False:
        print "============ Starting tutorial setup"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('movit_5dof',
                        anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("Test")
        display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)



        print "============ Generating plan 1"
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1
        pose_target.position.x = 0
        pose_target.position.y = 0.
        pose_target.position.z = 0.3
        group.set_pose_target(pose_target)
        plan1 = group.plan()
