function [qd,configSoln] = getIKJointAngles(pd,od,ik,initialguess,robot)

    rotm = eul2rotm(od');
    tform = blkdiag(rotm,1);
    tform(1:3,4) = pd;

    % qhome = [robot.homeConfiguration.JointPosition];
    % initialguess = robot.homeConfiguration;
    % initialguess(1).JointPosition = 0;
    % initialguess(2).JointPosition = -1.23;
    % initialguess(3).JointPosition = 1.7586;
    % initialguess(4).JointPosition = 1.04;
    % initialguess(5).JointPosition = -pi/2;

    posTgt = robotics.PositionTarget('right_finger');
    posTgt.TargetPosition = pd';
    
    orTgt = robotics.OrientationTarget('right_finger');
    orTgt.TargetOrientation = eul2quat(od');
    
    jointConst = robotics.JointPositionBounds(robot);
    ios = 0.05;
    jointConst.Bounds = [-pi+ios,pi-ios;
                    -pi,pi
                    -pi,pi
                    -pi,pi
                    -pi+ios,pi-ios];
    %Use lower weights for the orientation angles than the postion component
    weights = [0.25 0.25 0.25 1 1 1];
    %initialguess = robot.homeConfiguration;
%     [configSoln,solInfo] = ik('right_finger',tform,weights,initialguess,jointConst);
    [configSoln,solInfo] = ik(initialguess,jointConst,posTgt,orTgt);
    %show(robot,configSoln);

    qd = [configSoln(:).JointPosition]';
end