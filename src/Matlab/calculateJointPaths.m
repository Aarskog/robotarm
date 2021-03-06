function q_array = calculateJointPaths(path,od,robot,ik)
% Path is a cartesian path
n = size(path);

q_array = zeros(5,n(2));


initialguess = robot.homeConfiguration;

[initialq,initialguess]= getIKJointAngles(path(:,1),od,ik,initialguess,robot);

q_array(:,1) = initialq;

if n(2)>1
    for i=2:n(2)
        pd = path(:,i);
        [q_array(:,i),initialguess] = getIKJointAngles(pd,od,ik,initialguess,robot);
        fprintf('Inverse Kinematics running. %2.2f%% done \n',i/n(2)*100)
    end
end

end


 