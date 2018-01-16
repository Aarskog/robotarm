initialguess = robot.homeConfiguration;
%%
jointConst = robotics.JointPositionBounds(robot);
    ios = 0.005;
    jointConst.Bounds = [-pi+ios,pi-ios;
                    -pi,pi;
                    -pi,pi;
                    -pi,pi;
                    -pi+ios,pi-ios];

%asdf
for i = 0:0.01:0.2
od = [1.6; 0 ; pi];
pd = [i;-i;0.1]
                
                
posTgt = robotics.PositionTarget('right_finger');
posTgt.TargetPosition = pd';

orTgt = robotics.OrientationTarget('right_finger');
orTgt.TargetOrientation = eul2quat(od');

[configSoln,solInfo] = ik(initialguess,jointConst,posTgt,orTgt);
initialguess = configSoln;
pause(.1)
show(robotShow,configSoln);
end
getTransformationMatrix(T5,[configSoln(:).JointPosition]')