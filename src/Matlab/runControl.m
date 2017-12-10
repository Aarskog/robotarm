%% Run everytime Gazebo is (re)started
rosInit
%% Robot init
robotInit
generateForwardkinematics
%% Get kp kd
clc

%resetSim(gazebo);
%resumeSim(gazebo);
%jointStates = receive(jointstateSubscriber,10);
d1 = 0.0796;
a2 = 0.1347;
a3 = 0.0712;
d5 = 0.0918;


kp1 = 6;
kp2 = 6;
kp3 = kp2*1.1;
kp4 = kp2*1.5;
kp5 = .2;

Kp = [kp1,0,0,0,0;
      0,kp2,0,0,0;
      0,0,kp3,0,0;
      0,0,0,kp4,0;
      0,0,0,0,kp5];
    
kd1 = 1.9;
kd2 = 0.9;
kd3 = kd2;
kd4 = kd2;
kd5 = 0.1;
Kd =[kd1,0,0,0,0;
     0,kd2,0,0,0;
     0,0,kd3,0,0;
     0,0,0,kd4,0;
     0,0,0,0,kd5];

Kp = 1*Kp;
Kd = 1*Kd;
%% Gen path
steplength = 0.1;
t = 0:steplength:2*pi+steplength;%+2*steplength;

% Circle path zy plane
% ztest = 0.035*cos(t)+0.20;
% xtest = sin(t)*0;
% ytest = -0.04*sin(t)-0.20;
xpath = 0.035*cos(t)+0.09;%0.09;
ypath = 0.035*sin(t)+0.09;%0.09;
zpath = 0.050*sin(t)+0.25;
path = [xpath;ypath;zpath];
od = [pi*0/2; 0; 0];
%% Inverse Kin
tic
qarr = calculateJointPaths(path,od,robot,ik);
toc
ms_per_point = toc/length(qarr)*1000;
fprintf('Elapsed time per point: %2.2f ms \n',ms_per_point)
%% Run

%qdarr = estq;
qdarr = qarr;
% pd = [-0.2250;0;0.2250];
% tod = [-pi/2; 0; pi/2];
% pd = [0;0.2250;0.2250];
% tod = [0; pi/2; -pi/2];
% qdarr = calculateJointPaths(pd,tod,robot,ik);

% qdarr = qdarr + 0.05;

i = 2;
qd = qdarr(:,i);

iterations = 1;

tt = 7;


updateRate = robotics.Rate(100); 


disp('ᗡ== Controller is running ==D')
tic
timeiterators = ones(2,40);
timeiterator = 0;

%matrix containing datas q qdot qd u grav
datas = zeros(26,1);

reset(updateRate)
while toc < 2000
    %Frequency 
    timeiterator = timeiterator + 1;
    if timeiterator>length(timeiterators)
       timeiterator = 1;
    end
    timeiterators(:,timeiterator) = [iterations;toc];
    hertz = (max(timeiterators(1,:))-min(timeiterators(1,:)))/(max(timeiterators(2,:))-min(timeiterators(2,:)));
    fprintf('Controller is running. Update rate: \t %3.0f\tHz \n',hertz)

    %RupdateRate = double(1/(1/RupdateRate + 0.1*(1/updateRate-hertz)))

    q = jointstateSubscriber.LatestMessage.Position;  
    qdot = jointstateSubscriber.LatestMessage.Velocity;


    q([2 3 4]) = q([3 4 2]);
    qdot([2 3 4]) = qdot([3 4 2]);
    qtilde = qd-q;

    grav = gravityCompensation(q);
    u = Kp*qtilde - Kd*qdot + grav;
    u = torque_saturation(u,maxtorque);
    
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



    % Set next desired point at every tt iteration and when the end is
    % reached, start over. 
    if (mod(iterations,tt)==0)
        sizum = size(qdarr);
        if (i+1)>sizum(2)
            i=2;
        end
        qd = qdarr(:,i);
        i = i + 1;
        err = u-grav;
        debusdats =[err,grav,u];
    end
    iterations = iterations +1;
    
    datas = [datas,[q;qdot;qd;u;grav;toc]];
    waitfor(updateRate);
end
%% Super plot

superdatas =datas(:,2:end);
fontsize = 17;
figure(1)
plot(superdatas(26,:),superdatas(1:5,:))%-datas(11:15,:))
hold on
plot(superdatas(26,:),superdatas(11:15,:))
legs = legend({'$q_1$','$q_2$','$q_3$','$q_4$','$q_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('angle(rad)','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('Joint angles - large step','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on
hold off

figure(2)
plot(superdatas(26,:),superdatas(1:5,:)-superdatas(11:15,:))
legs = legend({'$q_1$','$q_2$','$q_3$','$q_4$','$q_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('angle(rad)','Interpreter','latex','FontSize',fontsize);
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('$$q-q_d$$ - large step','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on

figure(3)
plot(superdatas(26,:),superdatas(16:20,:))
legs = legend({'$u_1$','$u_2$','$u_3$','$u_4$','$u_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('u','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('u - large step','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on
   
figure(4)
plot(superdatas(26,:),superdatas(21:25,:))
legs = legend({'$g_1$','$g_2$','$g_3$','$g_4$','$g_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('g','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('Gravity components - large step','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on

figure(5)
plot(superdatas(26,:),superdatas(6:11,:))
legs = legend({'$\dot{q_1}$','$\dot{q_2}$','$\dot{q_3}$','$\dot{q_4}$','$\dot{q_5}$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('velocity(rad/s)','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('$$\dot{q}$$ - large step','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on
%% Plot ik paths
fontsize = 17;

figure(111)
show(robotShow)
linewdth = 2;
hold on
pr1 = plot3(path(1,:)*0,path(2,:),path(3,:),'LineWidth',linewdth);
pr2 = plot3(path(1,:),path(2,:)*0,path(3,:),'LineWidth',linewdth);
pr3 = plot3(path(1,:),path(2,:),path(3,:)*0,'LineWidth',linewdth);
pr4 = plot3(path(1,:),path(2,:),path(3,:),'LineWidth',linewdth);
hold off
legs = legend([pr1 pr2 pr3 pr4],{'yz-path','xz-path','xy-path','path'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('y(m)','Interpreter','latex','FontSize',fontsize);
xlabel('x(m)','Interpreter','latex','FontSize',fontsize);
zlabel('z(m)','Interpreter','latex','FontSize',fontsize);
tits = title('Desired path for end effector','interpreter','latex');
set(tits,'FontSize',fontsize);
zlim([0 inf])
xlim([-.2 .2])
ylim([-.2 .2])
%zlim([0 2*max(max(path(3,:)))])
grid on
%% Plot individual joint paths
figure(112)
for i = 1:5
 figure(111+i)
%  subplot(3,2,i)
 plot(qarr(i,2:end))
 xlim([0 length(qarr)])
 grid on
 hold on
 plot(t*0 +qarr(i,2))
 hold off
 titletext = sprintf('$$q_{%1.0f}$$',i);
 title(titletext,'interpreter','latex','FontSize',fontsize)
end
% plot(qarr(1,:))
% title('$$q_1$$','interpreter','latex','FontSize',fontsize);
%% Get xyz path of inverse kinematics solution
lp = length(qarr);
ikPath = zeros(3,lp);
parfor i=1:lp
    T = getTransformationMatrix(T5,qarr(:,i));
    ikPath(:,i) = T(1:3,4);
    fprintf('Running \t %2.2f %% done\n',i/lp*100)
end
%% Compare wanted path and ik generated path
fontsize = 17;

figure(118)
show(robotShow);
linewdth = 2;
hold on
pr11 = plot3(path(1,:),path(2,:),path(3,:),'LineWidth',linewdth);
pr12 = plot3( ikPath(1,1:end), ikPath(2,1:end), ikPath(3,1:end),'LineWidth',linewdth);
pr121 = plot3( ikPath(1,1:end), ikPath(2,1:end), ikPath(3,1:end)*0,'LineWidth',linewdth);
pr122 = plot3( ikPath(1,1:end), ikPath(2,1:end)*0, ikPath(3,1:end),'LineWidth',linewdth);
pr123 = plot3( ikPath(1,1:end)*0, ikPath(2,1:end), ikPath(3,1:end),'LineWidth',linewdth);
hold off
legs = legend([pr11 pr12 pr121 pr122 pr123],{'Desired Path','IK calculated path','IK xy-path','IK xz-path','IK yz-path'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('y(m)','Interpreter','latex','FontSize',fontsize);
xlabel('x(m)','Interpreter','latex','FontSize',fontsize);
zlabel('z(m)','Interpreter','latex','FontSize',fontsize);
tits = title('','interpreter','latex');
set(tits,'FontSize',fontsize);
zlim([0 inf]);
xlim([-.2 .2]);
ylim([-.2 .2]);
grid on;
%% See robot follow path
config =robot.homeConfiguration;

framerate = 15;
r = robotics.Rate(framerate);

figure(234)
show(robotShow)
hold on
for i=1:63
    for j=1:5
    config(j).JointPosition=qarr(j,i);
    end
    show(robotShow,config,'PreservePlot',false);
    waitfor(r);
end
hold off
%%
figure(311)
show(robotShow,config)

pr11 = plot3(path(1,:),path(2,:),path(3,:),'LineWidth',linewdth);
pr12 = plot3( ikPath(1,1:end), ikPath(2,1:end), ikPath(3,1:end),'LineWidth',linewdth);




