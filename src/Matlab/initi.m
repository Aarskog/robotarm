%% Init Remember to start Gazebo before running this
clear 

robot = importrobot('/home/magnaars/catkin_ws/src/five_dof_robotarm/urdf/matlabURDF.urdf');


% ik = robotics.InverseKinematics('RigidBodyTree',robot);

ik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot);
ik.ConstraintInputs={'joint','position','orientation'};

rosshutdown
pause(2)

rosinit
pause(1)

gazebo = ExampleHelperGazeboCommunicator();

%% Make a subscriber which will recive the joints states
jointstateSubscriber = rossubscriber('/five_dof_robotarm/joint_states');

% Create publishers which will publish desired joint torques to the
% different joint controllers
pub1 = rospublisher('/five_dof_robotarm/base_to_turntable_controller/command','std_msgs/Float64');
pub2 = rospublisher('/five_dof_robotarm/second_joint_controller/command','std_msgs/Float64');
pub3 = rospublisher('/five_dof_robotarm/third_joint_controller/command','std_msgs/Float64');
pub4 = rospublisher('/five_dof_robotarm/fourth_joint_controller/command','std_msgs/Float64');
pub5 = rospublisher('/five_dof_robotarm/fifth_joint_controller/command','std_msgs/Float64');

%Genereate messages which will be sent to the controller publishers
u1 = rosmessage(pub1);
u2 = rosmessage(pub2);
u3 = rosmessage(pub3);
u4 = rosmessage(pub4);
u5 = rosmessage(pub5);
pause(1)
