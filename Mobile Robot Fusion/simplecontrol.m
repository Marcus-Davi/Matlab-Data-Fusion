clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.15;
R = 10;
% [Xr,Ur,Tsim] = path_oito(R,v,Ts,[0 0 0]');
[Xr,Ur,Tsim] = path_S(15,v,Ts,[0 0 0]');
iterations = round((Tsim/Ts));

% Modelo do robô
R_real = 0.08; R_usado = 0.06;
D_real = 0.4; D_usado = 0.4;
Uncertainty.R = R_usado/R_real; %raio da roda
Uncertainty.D = D_usado/D_real; % distancia entre rodas



%% Controller Parameters
% Pi Controller
Kp = 1;
Ki = 100; %206.157 @ Ts = 0.1

%Feuler
pi_a = Kp+ Ki*Ts;
pi_b = Ki*Ts;
%% Gps Model

Lab0 = [-3.743718 -38.577979 0];
GPSRate = 5;
Ratio = (1/Ts)/GPSRate;
Gps_accu = 3;
Vel_accu = 0.1;
GPS = gpsSensor('UpdateRate',GPSRate,'ReferenceLocation',Lab0,'HorizontalPositionAccuracy',Gps_accu,'VelocityAccuracy',Vel_accu,'DecayFactor',0.5);

%% Kalman GPS
A = [1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
% x -> [x y x_ y_]
B = zeros(4,1);
C = eye(4);
D = 0;
SS = ss(A,B,C,D);
P = eye(4);
Qn = 0.01*diag([Gps_accu Gps_accu Vel_accu Vel_accu]);
% Qn = 0.0001*eye(4);
Rn = [Gps_accu^2 0 0 0;0 Gps_accu^2 0 0;0 0 Vel_accu^2 0;0 0 0 Vel_accu^2];

%% Kalman Robot
Pr = eye(3);
Qr = 0.0001*eye(3); %melhorar
Rr = diag([Gps_accu^2,Gps_accu^2,0.01]);

%% LQR CONTROLLER DESIGN
ur1 = 1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = 0;
SYS = ss(A,B,C,D);
Q = diag([1 1 0]);
R = 1*eye(2);
[K_LQ,S,E] = lqr(SYS,Q,R);


%% Simulation
yk = [-3 -3 0]';
yk_odo = yk;
yk_gps = yk;
ykalman = [yk(1:2);0;0;]; %inclui vx vy
ykalman_robot = yk;
u = [0.0 0]';
w = u(2);
e_t1 = 0; %past error


YK = zeros(iterations,3);
YK_ODO = zeros(iterations,3);
YK_GPS = [];
YK_Kalman = [];
YK_KalmanR = [];
Uk = zeros(iterations,2);

% GPS

for k=1:iterations
    
 
    % LOOP CLOSURE
    ykinput = [ykalman_robot(1) ykalman_robot(2) yk(3)]';
%     ykinput = [ykalman(1) ykalman(2) yk(3)]';
%     ykinput = yk_odo;
%     ykinput = [yk_gps(1) yk_gps(2) yk(3)]';
%     ykinput = yk;
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
    yk = robot_model(yk,u,Ts,Uncertainty); % real model + uncertainty
    p = [yk(2) yk(1) 0]; %formato NED
    v = [u(1)*cos(yk(3)) u(1)*sin(cos(yk(3))) 0];
    [gps_lla,v_gps] = GPS(p,v); % formato NED
    [gps_x,gps_y] = equiret(gps_lla(1),gps_lla(2),Lab0(1),Lab0(2));
    yk_gps = [gps_x gps_y]'; 

%     GPS Kalman
    if (rem(k,Ratio) == 0) % ASYNC UPDATE   
    YK_GPS = [YK_GPS;yk_gps'];
    [ykalman,P] = lkalman_predict(ykalman,0,P,Qn,SS);
    [ykalman,P] = lkalman_update([gps_x gps_y v_gps(1:2)]', ykalman,0,P,Rn,SS);    
    end
    
%     Robot Kalman
    [ykalman_robot,Pr] = ekalman_predict(ykalman_robot,u,Pr,Qr,@robot_model,@robot_jacobian,Ts);
    
    if (rem(k,Ratio) == 0) % ASYNC UPDATE
    [ykalman_robot,Pr] = ekalman_update([ykalman(1) ykalman(2) yk(3)]',ykalman_robot,u,Pr,Rr,@model_measurement,@measurement_jacobian,Ts);
    end
%     updates
    
    
    
    YK(k,:) = yk;
    YK_ODO(k,:) = yk_odo;
    YK_Kalman = [YK_Kalman;ykalman'];
    YK_KalmanR = [YK_KalmanR;ykalman_robot'];
    Uk(k,:) = u;
    
    
% plot(YK(1:k,1),YK(1:k,2),'b','linewidth',2)
% hold on
% plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
% hold off
% grid on
% drawnow
% pause(0.05)
    
end

%% Plots
close all

plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
hold on
plot(YK_ODO(1:k,1),YK_ODO(1:k,2),'linewidth',1)
plot(YK(1:k,1),YK(1:k,2),'linewidth',1)
plot(YK_GPS(:,1),YK_GPS(:,2),'r.','linewidth',2)
plot(YK_Kalman(:,1),YK_Kalman(:,2),'b.','linewidth',2)
plot(YK_KalmanR(:,1),YK_KalmanR(:,2),'g.','linewidth',2)
legend('Reference','Pure Odometry','Real Robot','GPS','GPS Kalman','GPS + Robot Kalman')
xlabel('X [m]')
ylabel('Y [m]')
grid on

return
E_GPS = YK(:,1:2) - YK_GPS;
E_Kalman = YK(:,1:2) - YK_Kalman(:,1:2);
disp('RMS RAW GPS')
disp(rms(E_GPS))

disp('RMS KALMAN')
    disp(rms(E_Kalman))
return
figure
plot(Uk(:,1))
hold on
plot(Uk(:,2))
legend('v','w')
% drawnow

    

