clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.15;
% R = 10;
% [Xr,Ur,Tsim] = path_oito(R,v,Ts,[0 0 0]');
[Xr,Ur,Tsim] = path_S(15,v,Ts,[0 0 0]');
iterations = round((Tsim/Ts));

% Modelo do robô
R_real = 0.08; R_usado = 0.1;
D_real = 0.4; D_usado = 0.38;
Uncertainty.R = R_usado/R_real; %raio da roda
Uncertainty.D = D_usado/D_real; % distancia entre rodas

%% Gps Model

Lab0 = [-3.743718 -38.577979 0];
GPSRate = 1; % Hz
Ratio = (1/Ts)/GPSRate;
Gps_accu = 3;
Vel_accu = 0.1;
GPS = gpsSensor('UpdateRate',GPSRate,'ReferenceLocation',Lab0,'HorizontalPositionAccuracy',Gps_accu,'VelocityAccuracy',Vel_accu,'DecayFactor',0.5);

%% Kalman GPS
% x -> [x y x_ y_]
A = [1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
B = zeros(4,1);
C = eye(4);
D = 0;
SS = ss(A,B,C,D);
P = 10000*eye(4);
Qn = 0.01*diag([Gps_accu Gps_accu Vel_accu Vel_accu]); %aqui tem que ser brown motion. qt maior, mais ele tende pro unified
Rn = 1*diag([Gps_accu^2 Gps_accu^2 Vel_accu^2 Vel_accu^2]);

%% Kalman Robot
Pr = 100000*eye(3);
Qr = 0.0001*eye(3); % melhorar projeto. 0.0001 tá ok
Rr = diag([Gps_accu^2,Gps_accu^2,0.01]);

%% Kalman Unified
Pr_unified = Pr;
Qr_unified = Qr;
Rr_unified = Rr;

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
yk = [-3 -3 0]';
yk_odo = yk;
yk_gps = yk;
ykalman_gps = [yk(1:2);0;0;]; %inclui vx vy
ykalman_robot = yk;

ykalman_unified = yk;
u = [0.0 0]';


YK = zeros(iterations,3);
YK_ODO = zeros(iterations,3);
YK_GPS = [];
YK_KalmanG = [];
YK_KalmanR = [];
YK_KalmanU = [];
Uk = zeros(iterations,2);

% GPS

for k=1:iterations
    
 
    % Control loop closure
%     ykinput = [ykalman_unified(1) ykalman_unified(2) yk(3)]';
      ykinput = [ykalman_robot(1) ykalman_robot(2) yk(3)]'; %mt mais suave
%     ykinput = [ykalman_gps(1) ykalman_gps(2) yk(3)]';
%     ykinput = yk_odo; %odo
%     ykinput = [yk_gps(1) yk_gps(2) yk(3)]'; 
%     ykinput = yk; % true value

    % Control feedback
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
    
  
        if v > 1
            v = 1;
        elseif v < 0
            v = 0;
        end
        
        if w > 10
            w = 10;
        elseif w < -10
            w = -10;
        end
    
    u = [v w]';
    
    
    yk_odo = robot_model(yk_odo,u,Ts); % odometry model
    
    
    yk = robot_model(yk,u,Ts,Uncertainty); % real model + uncertainty | TRUE VALUE
    
    
    p = [yk(2) yk(1) 0]; %formato NED
    v = [u(1)*cos(yk(3)) u(1)*sin(cos(yk(3))) 0]; %true velocity
    [gps_lla,v_gps] = GPS(p,v); % formato NED
    [gps_x,gps_y] = equiret(gps_lla(1),gps_lla(2),Lab0(1),Lab0(2));
    yk_gps = [gps_x gps_y]'; % GPS Convertido p X,Y

%     GPS Kalman
    if (rem(k,Ratio) == 0) % ASYNC UPDATE   
    YK_GPS = [YK_GPS;yk_gps'];
    [ykalman_gps,P] = lkalman_predict(ykalman_gps,0,P,Qn,SS);
    [ykalman_gps,P] = lkalman_update([gps_x gps_y v_gps(1:2)]', ykalman_gps,0,P,Rn,SS);    
    end
    
%     Robot Kalman
    [ykalman_robot,Pr] = ekalman_predict(ykalman_robot,u,Pr,Qr,@robot_model,@robot_jacobian,Ts);
    [ykalman_unified,Pr_unified] = ekalman_predict(ykalman_unified,u,Pr_unified,Qr,@robot_model,@robot_jacobian,Ts);
    
    if (rem(k,Ratio) == 0) % ASYNC UPDATE
    [ykalman_robot,Pr] = ekalman_update([ykalman_gps(1) ykalman_gps(2) yk(3)]',ykalman_robot,u,Pr,Rr,@model_measurement,@measurement_jacobian,Ts);
    [ykalman_unified,Pr_unified] = ekalman_update([gps_x gps_y yk(3)],ykalman_unified,u,Pr_unified,Rr,@model_measurement,@measurement_jacobian,Ts);
    end
%     updates
    
    
    
    YK(k,:) = yk;
    YK_ODO(k,:) = yk_odo;
    YK_KalmanG = [YK_KalmanG;ykalman_gps'];
    YK_KalmanR = [YK_KalmanR;ykalman_robot'];
    YK_KalmanU = [YK_KalmanU;ykalman_unified'];
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
e_odo = YK - YK_ODO;
e_kalman_gps = YK(:,1:2) - YK_KalmanG(:,1:2);
e_kalman_cascade = YK - YK_KalmanR;
e_kalman_unified = YK - YK_KalmanU;

disp('RMS ODO')
disp(rms(e_odo))

disp('RMS GPS')
disp(rms(e_kalman_gps))

disp('RMS CASCADE')
disp(rms(e_kalman_cascade))

disp('RMS UNIFIED')
disp(rms(e_kalman_unified))


%% Plots
close all

plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
hold on
plot(YK_ODO(1:k,1),YK_ODO(1:k,2),'linewidth',2)
plot(YK(1:k,1),YK(1:k,2),'linewidth',2)
plot(YK_GPS(:,1),YK_GPS(:,2),'r.','linewidth',2)
plot(YK_KalmanG(:,1),YK_KalmanG(:,2),'b.','linewidth',2)
plot(YK_KalmanR(:,1),YK_KalmanR(:,2),'g','linewidth',2)
plot(YK_KalmanU(:,1),YK_KalmanU(:,2),'m','linewidth',2)
legend('Reference','Pure Odometry','Real Robot','GPS','GPS Kalman','Cascade Kalman','Unified Kalman')
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



    

