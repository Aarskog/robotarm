clc
%resetSim(gazebo);
%% Run things
updateRate = 0.01;
%resetSim(gazebo);
%resumeSim(gazebo);
%jointStates = receive(jointstateSubscriber,10);
d1 = 0.0796;
a2 = 0.1347;
a3 = 0.0712;
d5 = 0.0918;


kp1 = 1.4;
kp2 = 6;
kp3 = kp2*1.1;
kp4 = kp2*1.5;
kp5 = 10;

Kp = [kp1,0,0,0,0;
    0,kp2,0,0,0;
    0,0,kp3,0,0;
    0,0,0,kp4,0;
    0,0,0,0,kp5];
    
kd1 = 1;
kd2 = 0.9;
kd3 = kd2;
kd4 = kd2;
kd5 = 1;
Kd =[kd1,0,0,0,0;
    0,kd2,0,0,0;
    0,0,kd3,0,0;
    0,0,0,kd4,0;
    0,0,0,0,kd5];

Kp = 1*Kp;
Kd = 1*Kd;

%Create desired position and orientation
pd = [0.2;0;d1+a2+a3-0.1];
od = [0; pi/2; pi/2];

%Create tranformation matrix from desired position and orientation
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

%%
%Use lower weights for the orientation angles than the postion component
weights = [0.25 0.25 0.25 1 1 1];
initialguess = robot.homeConfiguration;
[configSoln,solInfo] = ik('right_finger',tform,weights,initialguess);
show(robot,configSoln);

qd = [configSoln(:).JointPosition]';

%% asdf


while 1
   q = jointstateSubscriber.LatestMessage.Position;  
   qdot = jointstateSubscriber.LatestMessage.Velocity;
   
   q([2 3 4]) = q([3 4 2]);
   qdot([2 3 4]) = qdot([3 4 2]);
   qtilde = qd-q;
   
   u = Kp*qtilde - +Kd*qdot + gravityCompensation(q);

   u1.Data = u(1);
   u2.Data = u(2);
   u3.Data = u(3);
   u4.Data = u(4);
   u5.Data = u(5);
   
   send(pub1,u1)
   send(pub2,u2)
   send(pub3,u3)
   send(pub4,u4)
   send(pub5,u5)
   
   
   pause(updateRate)
end