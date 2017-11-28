function [qd,configSoln] = getIKJointAngles(pd,od,ik,initialguess,robot)

    rotm = eul2rotm(od');
    tform = blkdiag(rotm,1);
    tform(1:3,4) = pd;

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