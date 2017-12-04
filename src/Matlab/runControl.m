%% Run everytime Gazebo is (re)started
rosInit
%%
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
kp5 = 2;

Kp = [kp1,0,0,0,0;
    0,kp2,0,0,0;
    0,0,kp3,0,0;
    0,0,0,kp4,0;
    0,0,0,0,kp5];
    
kd1 = 1.9;
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
%% Gen path
t = 0:0.1:2*pi;

ztest = 0.035*cos(t)+0.20;
xtest = sin(t)*0;
ytest = -0.04*sin(t)-0.20;
path = [xtest;ytest;ztest];


od = [0; pi/2; pi/2];
%% tic
qarr = calculateJointPaths(path,od,robot,ik);
qarr(5,:) = qarr(5,:)*0+pi/2;
% toc

%% 
figure(5783)
for i=1:5
    subplot(5,1,i)
    plot(qarr(i,:))
end

%% Run

qdarr = estq;
qdarr = qarr;
% pd = [-0.2250;0;0.2250];
% tod = [-pi/2; 0; pi/2];
% pd = [0;0.2250;0.2250];
% tod = [0; pi/2; -pi/2];
% qdarr = calculateJointPaths(pd,tod,robot,ik);

qdarr = qdarr + 0.05;
qd = qdarr(:,1);
i = 1;
iterations = 1;

tt = 5;


updateRate = 0.01; 
RupdateRate = updateRate/2;

disp('ᗡ== Controller is running ==D')
tic
timeiterators = ones(2,200);
timeiterator = 0;

%matrix containing datas q qdot qd u grav
datas = zeros(26,1);


while toc < 2000
    %Frequency 
    timeiterator = timeiterator + 1;
    if timeiterator>length(timeiterators)
       timeiterator = 1;
    end
    timeiterators(:,timeiterator) = [iterations;toc];
    hertz = (max(timeiterators(1,:))-min(timeiterators(1,:)))/(max(timeiterators(2,:))-min(timeiterators(2,:)))

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

    pause(RupdateRate)

    % Set next desired point at every tt iteration and when the end is
    % reached, start over. 
    if (mod(iterations,tt)==0)
        sizum = size(qdarr);
        if (i+1)>sizum(2)
            i=1;
        end
        qd = qdarr(:,i);
        i = i + 1;
        err = u-grav;
        debusdats =[err,grav,u];
    end
    iterations = iterations +1;
    
    datas = [datas,[q;qdot;qd;u;grav;toc]];
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


%%
hust = superdatas(26,2:end)-superdatas(26,1:end-1);
std((hust))
