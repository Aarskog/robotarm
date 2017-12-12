%% Run everytime Gazebo is (re)started
rosInit
%% Robot init
robotInit
generateForwardkinematics
%% Get kp kd
clc

d1 = 0.0796;
a2 = 0.1347;
a3 = 0.0712;
d5 = 0.0918;


kp1 = 6;
kp2 = 6;
kp3 = kp2*1.1;
kp4 = kp2*1.5;
kp5 = .3;

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
Kd = 0.9*Kd;
%% Gen path
steplength = 0.03;
t = 0:steplength:2*pi+steplength;%+2*steplength;

% Circle path zy plane
% zpath = 0.035*cos(t)+0.20;
% xpath = sin(t)*0;
% ypath = -0.04*sin(t)-0.20;
% od = [0; 0; pi/2];

xpath = 0.035*cos(t)+0.09;%0.09;
ypath = 0.035*sin(t)+0.09;%0.09;
zpath = 0.050*sin(t)+0.25;
od = [pi*0/2; 0; 0];

path = [xpath;ypath;zpath];
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
% qdarr = calculateJointPaths([pd pd],tod,robot,ik);

% qdarr = qdarr + 0.05;

i = 2;
qd = qdarr(:,i);

iterations = 1;

tt = 4;
coa = 0.2;

rossrate = 180;
updateRate = robotics.Rate(rossrate); 

dt = tt/(rossrate);

disp('ᗡ== Controller is running ==D')

timeiterators = ones(2,40);
timeiterator = 0;

%matrix containing datas q qdot qd u grav
datas = zeros(31,1);
tic
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

    %Get the latest robot state
    q = jointstateSubscriber.LatestMessage.Position;  
    qdot = jointstateSubscriber.LatestMessage.Velocity;

    %The joint state publisher is sending back scrambled joints states
    q([2 3 4]) = q([3 4 2]);
    qdot([2 3 4]) = qdot([3 4 2]);
    
    %Get the next and last desired set point
    qdnext = getqdNext(qdarr,i-1,2);
    qdlast = getqdLast(qdarr,i-1,1);
    
    %Calculate the desired joint velocities based on the slope of the set
    %points
    vk = (qd-qdlast)/dt;
    vkp1 = (qdnext(:,1)-qd)/dt;
    qdotd = 1/2*(vk+vkp1).*(sign(vk)==sign(vkp1));
    
    qtilde = (qd-q)/1 + (qdnext(:,1)-q)/0.8+(qdnext(:,2)-q)/1;

    grav = gravityCompensation(q);
    u = Kp*qtilde - Kd*(qdot-qdotd) + grav;
    u = torque_saturation(u,maxtorque);
    
    %Create and send joint torques
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

    datas = [datas,[q;qdot;qd;u;grav;toc;qdotd]];

    % Set next desired point at every tt iteration and when the end is
    % reached, start over. 
    if (mod(iterations,tt)==0)
        sizum = size(qdarr);
        if (i+1)>sizum(2)
            i=2;
        end
        qd = qdarr(:,i);
        qd = someKindOfFeedForward(qdarr,i);
        i = i + 1;
        err = u-grav;
        debusdats =[err,grav,u];
    end
    iterations = iterations +1;
    
%     if norm(qd-q)<coa
%         sizum = size(qdarr);
%         if (i+1)>sizum(2)
%             i=2;
%         end
%         qd = qdarr(:,i);
%         qd = someKindOfFeedForward(qdarr,i);
%         i = i + 1;
%         err = u-grav;
%         debusdats =[err,grav,u];
%     end
    
    
    waitfor(updateRate);
end


%% Plotting
%% Super plot

superdatas =datas(:,2:end);
fontsize = 17;
figure(1)
plot(superdatas(26,:),superdatas(1:5,:))%-datas(11:15,:))
%hold on
%plot(superdatas(26,:),superdatas(11:15,:))
legs = legend({'$q_1$','$q_2$','$q_3$','$q_4$','$q_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('angle(rad)','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('Joint angles','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on
%hold off

figure(2)
plot(superdatas(26,:),superdatas(1:5,:)-superdatas(11:15,:))
legs = legend({'$q_1$','$q_2$','$q_3$','$q_4$','$q_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('angle(rad)','Interpreter','latex','FontSize',fontsize);
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('$$q-q_d$$','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on

figure(3)
plot(superdatas(26,:),superdatas(16:20,:))
legs = legend({'$u_1$','$u_2$','$u_3$','$u_4$','$u_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('u','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('u ','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on
   
figure(4)
plot(superdatas(26,:),superdatas(21:25,:))
legs = legend({'$g_1$','$g_2$','$g_3$','$g_4$','$g_5$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('g','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('Gravity components ','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on

figure(5)
plot(superdatas(26,:),superdatas(6:11,:))
legs = legend({'$\dot{q_1}$','$\dot{q_2}$','$\dot{q_3}$','$\dot{q_4}$','$\dot{q_5}$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('velocity(rad/s)','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('$$\dot{q}$$','interpreter','latex');
set(tits,'FontSize',fontsize);
grid on

figure(6)
plot(superdatas(26,:),superdatas(27:31,:))
legs = legend({'$\dot{q_{d_1}}$','$\dot{q_{d_2}}$','$\dot{q_{d_3}}$','$\dot{q_{d_4}}$','$\dot{q_{d_5}}$'},'Interpreter','latex');
set(legs,'FontSize',fontsize);
ylabel('velocity(rad/s)','Interpreter','latex','FontSize',fontsize)
xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
tits = title('$$\dot{q_{d_i}}$$','interpreter','latex');
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
%% Plot desired joint paths and actual joint pahts - 5 individual plots
%datas = [datas,[q;qdot;qd;u;grav;toc]];
superdatas =datas(:,2:end);
fontsize = 17;

for i=1:nJoints
    figure(44+i)
    plot(superdatas(26,:),superdatas(i,:))
    hold on
    plot(superdatas(26,:),superdatas(10+i,:))
    hold off
    grid on
    legendarytext1 = sprintf('$q_{%1.0f}$',i);
    legendarytext2 = sprintf('$q_{d%1.0f}$',i);
    legend({legendarytext1,legendarytext2},'Interpreter','latex','FontSize',fontsize)
    ylabel('angle(rad)','Interpreter','latex','FontSize',fontsize)
    xlabel('time(sec)','Interpreter','latex','FontSize',fontsize)
end








