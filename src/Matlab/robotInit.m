
robot = importrobot('/home/magnaars/catkin_ws/src/five_dof_robotarm/urdf/matlabURDF.urdf');

%Make another robot for showcase
robotShow = importrobot('/home/magnaars/catkin_ws/src/five_dof_robotarm/urdf/matlabURDFShow.urdf');

maxtorque = 7.3;
nJoints = 5;

% ik = robotics.InverseKinematics('RigidBodyTree',robot);

ik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot);
ik.ConstraintInputs={'joint','position','orientation'};
%ik.SolverAlgorithm = 'LevenbergMarquardt';
ik.SolverParameters.SolutionTolerance = 0.00001;
ik.SolverParameters.GradientTolerance = 0.000001;
ik.SolverParameters.MaxTime = 0.1;
ik.SolverParameters.AllowRandomRestart = 0;


% gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot);
% gik.ConstraintInputs={'joint','position','orientation'};
% gik.SolverAlgorithm = 'LevenbergMarquardt';
% gik.SolverParameters.SolutionTolerance = 0.00001;
% gik.SolverParameters.GradientTolerance = 0.000001;
% gik.SolverParameters.MaxTime = 0.1;
% gik.SolverParameters.AllowRandomRestart = 0;
