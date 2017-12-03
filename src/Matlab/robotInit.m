
robot = importrobot('/home/magnaars/catkin_ws/src/five_dof_robotarm/urdf/matlabURDF.urdf');

maxtorque = 7.3;

% ik = robotics.InverseKinematics('RigidBodyTree',robot);

ik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot);
ik.ConstraintInputs={'joint','position','orientation'};
%ik.SolverAlgorithm = 'LevenbergMarquardt';
ik.SolverParameters.SolutionTolerance = 0.00001;
ik.SolverParameters.GradientTolerance = 0.000001;
ik.SolverParameters.MaxTime = 1;
ik.SolverParameters.AllowRandomRestart = 0;
