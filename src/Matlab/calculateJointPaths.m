function q_array = calculateJointPaths(path,od,robot,ik)
 % Path is a cartesian path
 n = length(path);
 q_array = zeros(5,n);
 

 xy = [0.2;0];
 
 initialguess = robot.homeConfiguration;
 [initialq,initialguess]= getIKJointAngles([xy;path(1)],od,ik,initialguess,robot);

 
 %Change to a faster solver
%  release(ik)
%  ik = robotics.InverseKinematics('RigidBodyTree',robot);
%  ik.SolverAlgorithm ='LevenbergMarquardt';
%  
 q_array(:,1) = initialq;
 for i=2:n
    pd = [xy;path(i)];
    [q_array(:,i),initialguess] = getIKJointAngles(pd,od,ik,initialguess,robot);
    %q_array(:,i) = mod((q_array(:,i) + pi),2*pi)-pi;
    fprintf('Inverse Kinematics running. %2.2f%% done \n',i/n*100)
 end

end


 