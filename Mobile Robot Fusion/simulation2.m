clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.15; %track speed
% R = 10;
% [Xr,Ur,Tsim] = path_oito(R,v,Ts,[0 0 0]');
[Xr,Ur,Tsim] = path_S(15,v,Ts,[0 0 0]');
iterations = round((Tsim/Ts));

% Modelo do robô
R_real = 0.08; R_usado = 0.11; %0.1
D_real = 0.4; D_usado = 0.38; %0.38
Uncertainty.R = R_usado/R_real; %raio da roda
Uncertainty.D = D_usado/D_real; % distancia entre rodas

%% Gps Model

Lab0 = [-3.743718 -38.577979 0];
GPSRate = 1; % Hz
Ratio = (1/Ts)/GPSRate;
Gps_accu = 2; %Accuracy (meters)
Vel_accu = 1; % Accuracy (m)
GPS = gpsSensor('UpdateRate',GPSRate,'ReferenceLocation',Lab0,'HorizontalPositionAccuracy',Gps_accu,'VelocityAccuracy',Vel_accu,'DecayFactor',0.5);

%% Extended Kalman
% Model is Non Linear

P = 0.1*eye(6);
Qn = 0.01*[Ts^3/3 Ts^2/2 0 0 0 0;
    Ts^2/2 Ts 0 0 0 0;
    0 0 Ts^3/3 Ts^2/2 0 0;
    0 0 Ts^2/2 Ts 0 0;
    0 0 0 0 Ts^3/3 Ts^2/2;
    0 0 0 0 Ts^2/2 Ts];

Rn = 1*diag([Gps_accu^2 Vel_accu^2 Gps_accu^2 Vel_accu^2 0.001]);

%% LQR CONTROLLER DESIGN
ur1 = v;
ur2 = 0.0;
A_lq = [0 ur2 0;-ur2 0 ur1;0 0 0];
B_lq = [1 0;0 0;0 1];
C_lq = eye(3);
D_lq = 0;
SYS = ss(A_lq,B_lq,C_lq,D_lq);
Q_lq = diag([1 1 0]);
R_lq = 1*eye(2);
[K_LQ,S,E] = lqr(SYS,Q_lq,R_lq);


%% Simulation
yk = [-3 0 -3 0 0 0]'; %state vector
yk_odo = yk;
yk_gps = yk;
ykalman_robot = yk;
u = [0.0 0]';


YK = zeros(iterations,length(yk));
YK_ODO = zeros(iterations,length(yk)); %hard
YK_GPS = [];
YK_Kalman = [];
Uk = zeros(iterations,length(u));

% GPS

for k=1:iterations
    
    
    % Control loop closure
    
%           ykinput = [ykalman_robot(1) ykalman_robot(3) ykalman_robot(5)]'; %mt mais suave
    %     ykinput = yk_odo; %odo
    %     ykinput = [yk_gps(1) yk_gps(2) yk(3)]';
    ykinput = [yk(1) yk(3) yk(5)]'; % true value [x y theta]
    
    % Start of control law
    e = Xr(:,k) - ykinput; %feedback aqui
    
    T = [cos(ykinput(3)) sin(ykinput(3)) 0;
        -sin(ykinput(3)) cos(ykinput(3)) 0;
        0 0 1];
    e_body = T*e;
    
    V = -K_LQ*e_body;
    v1 = V(1);
    v2 = V(2);
    
    v = Ur(1,k)*cos(e_body(3)) - v1;
    w = Ur(2,k) -  v2;
    
    
    if v > 0.2
        v = 0.2;
    elseif v < 0
        v = 0;
    end
    
    if w > 1
        w = 1;
    elseif w < -1
        w = -1;
    end
    
    u = [v w]';
    
    % End of control law
    
    
    yk_odo = robot_model2(yk_odo,u,Ts); % odometry model
    
    
    yk = robot_model2(yk,u,Ts,Uncertainty); % real model + uncertainty | TRUE VALUE
    
    [ykalman_robot,P] = ekalman_predict(ykalman_robot,u,P,Qn,@robot_model2,@robot_jacobian2,Ts);
    
    
    p = [yk(3) yk(1) 0]; % formato NED
    v = [yk(4) yk(2) 0]; % formato NED
    [gps_lla,v_gps] = GPS(p,v); % formato NED
    [gps_x,gps_y] = equiret(gps_lla(1),gps_lla(2),Lab0(1),Lab0(2));
    yk_gps = [gps_x gps_y]'; % GPS Convertido p X,Y
      
    if (rem(k,Ratio) == 0) % ASYNC UPDATE
        YK_GPS(k,:) = yk_gps;
        % measured -> [x,y,x_,y_,th]
        yk_measurement = [gps_x v_gps(2) gps_y v_gps(1) yk(5)]' %v_gps(2) -> east = x
        [ykalman_robot,P] = ekalman_update(yk_measurement,ykalman_robot,u,P,Rn,@model_measurement2,@measurement_jacobian2,Ts);
    end
    %     updates
    
    
    
    YK(k,:) = yk;
    YK_ODO(k,:) = yk_odo;
    
    YK_Kalman = [YK_Kalman;ykalman_robot'];
    Uk(k,:) = u;
    
    
    % plot(YK(1:k,1),YK(1:k,2),'b','linewidth',2)
    % hold on
    % plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
    % hold off
    % grid on
    % drawnow
    % pause(0.05)
    
end

%% Errors
% e_odo = YK - YK_ODO;
e_kalman = YK(:,[1 3 5]) - YK_Kalman(:,[1 3 5]);



disp('RMS GPS')
disp(rms(e_kalman))


%% Plots
close all

plot(YK_GPS(:,1),YK_GPS(:,2),'r.','linewidth',2)
hold on
plot(YK_ODO(1:k,1),YK_ODO(1:k,3),'linewidth',2)
plot(YK(1:k,1),YK(1:k,3),'linewidth',2)
plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
plot(YK_Kalman(:,1),YK_Kalman(:,3),'b','linewidth',2)
legend('GPS Raw Data','Pure Odometry','Real Robot','Reference','Kalman')
xlabel('X [m]')
ylabel('Y [m]')
grid on

figure
plot(Uk(:,1))
hold on
plot(Uk(:,2))
legend('v','w')
grid on
return





