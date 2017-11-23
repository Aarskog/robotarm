clc
robot = importrobot('/home/magnaars/catkin_ws/src/five_dof_robotarm/urdf/matlabURDF.urdf');

%%
ik = robotics.InverseKinematics('RigidBodyTree',robot);

%%
rosinit