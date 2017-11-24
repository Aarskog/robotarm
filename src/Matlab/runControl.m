clc
%resetSim(gazebo);
initi
generateForwardkinematics
%% Run things
clc

%resetSim(gazebo);
%resumeSim(gazebo);
%jointStates = receive(jointstateSubscriber,10);
d1 = 0.0796;
a2 = 0.1347;
a3 = 0.0712;
d5 = 0.0918;


kp1 = 2.4;
kp2 = 6;
kp3 = kp2*1.1;
kp4 = kp2*1.5;
kp5 = 3;

Kp = [kp1,0,0,0,0;
    0,kp2,0,0,0;
    0,0,kp3,0,0;
    0,0,0,kp4,0;
    0,0,0,0,kp5];
    
kd1 = 1;
kd2 = 0.9;
kd3 = kd2;
kd4 = kd2;
kd5 = 0.9;
Kd =[kd1,0,0,0,0;
    0,kd2,0,0,0;
    0,0,kd3,0,0;
    0,0,0,kd4,0;
    0,0,0,0,kd5];

Kp = 1*Kp;
Kd = 1*Kd;

ztest = 0.035*sin(0:0.2:4*pi)+0.1886;
%plot(ztest);
od = [0; pi/2; pi/2];

qarr = calculateJointPaths(ztest,od,robot,ik);

%
% %Create desired position and orientation
% pd = [0.2;0;d1+a2+a3-0.1];
% od = [0; pi/2; pi/2];
% 
% %Create tranformation matrix from desired position and orientation
% qd = getIKJointAngles(pd,od,ik,robot.homeConfiguration);

%% show configuration
config = robot.homeConfiguration;
config(1).JointPosition = qarr(1,end);
config(2).JointPosition = qarr(2,end);
config(3).JointPosition = qarr(3,end);
config(4).JointPosition = qarr(4,end);
config(5).JointPosition = qarr(5,end);
show(robot,config)

%% 
tempest = qarr;
%%
clc
% qarr(1,:) = 0;
a = 2;

figure(345)
plot(medfilt1(sign(qarr(a,1)).*abs(mod((qarr(a,:) + pi),2*pi)-pi),1))
hold on
plot(0.24*sin(0:0.2:4*pi)-1.21)
hold off

estq = zeros(5,length(qarr));

estq(1,:) = qarr(1,:);
estq(2,:) = 0.24*sin(0:0.2:4*pi)-1.21;
estq(3,:) = sign(qarr(3,1)).*abs(mod((qarr(3,:) + pi),2*pi)-pi);
estq(4,:) = 0.2*sin(0:0.2:4*pi)+1.1;
estq(5,:) = 0*qarr(5,:);

% for i = 1:5
%     a=i;
%     estq(i,:) = medfilt1(sign(qarr(a,1)).*abs(mod((qarr(a,:) + pi),2*pi)-pi),9);
% end
% estq = medfilt1(sign(qarr(a,1)).*abs(mod((qarr(a,:) + pi),2*pi)-pi),9);
%% 
b = 2;
figure(2344)
plot(qarr(b,:))
hold on
plot(estq(b,:))
hold off

%% asdf
qdarr = estq;

qd = qdarr(:,1);
i = 1;
iterations = 1;

tt = 10;

erroracc = 0.05;
updateRate = 0.01;
while 1
   q = jointstateSubscriber.LatestMessage.Position;  
   qdot = jointstateSubscriber.LatestMessage.Velocity;
   
%    dk = directKinematics(T5,q);
%    dkd = directKinematics(T5,qd);
   
%    error = norm(dk-dkd);
   
%    if error<erroracc
%       i=i+1;
%       if i>=length(qarr)
%          i=1; 
%       end
%       qd = qarr(:,i);
%    end
%    
   q([2 3 4]) = q([3 4 2]);
   qdot([2 3 4]) = qdot([3 4 2]);
   qtilde = qd-q;
   
   u = Kp*qtilde - Kd*qdot + gravityCompensation(q);

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

   if mod(iterations,tt)==0
       i = i + 1
       if i>=length(qarr)
         i=1; 
       end
      qd = qdarr(:,i);
   end
   iterations = iterations +1
end