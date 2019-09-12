clear;close all;clc
Ts_imu = 0.02;

A_imu = [1 0 Ts_imu 0 Ts_imu^2/2 0;
    0 1 0 Ts_imu 0 Ts_imu^2/2;
    0 0 1 0 Ts_imu 0;
    0 0 0 1 0 Ts_imu;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

C_imu = [0 0 0 0 1 0;0 0 0 0 0 1];

% B_imu = zeros(6,1);

Rv_imu = [0.01 0;0 0.01];
Rw_imu = 1*[Ts_imu^5/20 0 Ts_imu^4/8 0 Ts_imu^3/6 0; %Process
             0 Ts_imu^5/20 0 Ts_imu^4/8 0 Ts_imu^3/6;
             Ts_imu^4/8 0 Ts_imu^3/3 0 Ts_imu^2/2 0;
             0 Ts_imu^4/8 0 Ts_imu^3/3 0 Ts_imu^2/2;
             Ts_imu^3/6 0  Ts_imu^2/2 0  Ts_imu 0;
             0 Ts_imu^3/6 0 Ts_imu^2/2 0 Ts_imu];


Ts_gps = 1;

A_gps = [1 0 Ts_gps 0 Ts_gps^2/2 0;
    0 1 0 Ts_gps 0 Ts_gps^2/2;
    0 0 1 0 Ts_gps 0;
    0 0 0 1 0 Ts_gps;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

B_gps = zeros(6,1);
C_gps = [1 0 0 0 0 0;0 1 0 0 0 0];


Rv_gps = 1*eye(2);
Rw_gps = 1*[Ts_gps^5/20 0 Ts_gps^4/8 0 Ts_gps^3/6 0; %Process
             0 Ts_gps^5/20 0 Ts_gps^4/8 0 Ts_gps^3/6;
             Ts_gps^4/8 0 Ts_gps^3/3 0 Ts_gps^2/2 0;
             0 Ts_gps^4/8 0 Ts_gps^3/3 0 Ts_gps^2/2;
             Ts_gps^3/6 0  Ts_gps^2/2 0  Ts_gps 0;
             0 Ts_gps^3/6 0 Ts_gps^2/2 0 Ts_gps];


%% Algoritmo

t = 0:Ts_imu:20;
t_gps = [];
y_imu = [t >= 1; t>=1];
x_imu = zeros(6,1); %imu states
% y_imu = zeros(2,1); %imu reading(accelerometer)

gps_noise = rand(2,length(t))*Rv_gps(1) - Rv_gps(1)/2;
y_gps = [t>=5; t>=5] + gps_noise;
x_gps = zeros(6,1); %gps states

X_imu = [];
X_gps = [];
%   FAST LOOP
%       PREDICT IMU (USE LAST GPS DATA)
%       UPDATE IMU 
%   FAST LOOP END    
% SLOW LOOP
%  PREDICT GPS (USE LAST IMU DATA)
%  UPDATE GPS
% SLOW LOOP END

    Pk_imu = zeros(6);
    Pk_gps = zeros(6);
    i = 1;
    
    GPS_DATA_FLAG = 0;
    for ta = t
        
        
        
        %     SLOW LOOP
    if(rem(i-1,50) == 0)
    %predict
    x_gps = A_gps*x_gps; %1
    Pk_gps = A_gps*Pk_gps*A_gps' + Rw_gps; %2
    
    %update
    K_gps = (Pk_gps*C_gps')/(C_gps*Pk_gps*C_gps' + Rv_gps); %3
    x_gps = x_gps + K_gps*(y_gps(:,i)-C_gps*x_gps); %4
    Pk_gps = (eye(size(K_gps,1)) - K_gps*C_gps)*Pk_gps; %5
    X_gps = [X_gps x_gps];
    t_gps = [t_gps ta];
    GPS_DATA_FLAG = 1;
    end
    
    
        %FAST LOOP
        %predict
    x_imu = A_imu*x_imu ;   %1
    Pk_imu = A_imu*Pk_imu*A_imu' + Rw_imu; %2 
        %update
    K_imu = (Pk_imu*C_imu')/(C_imu*Pk_imu*C_imu' + Rv_imu); %3
    if(GPS_DATA_FLAG==1)
%     x_imu = x_imu+K_imu*(y_imu(:,i)-C_imu*x_imu); %4 XHAT
%     x_imu = x_imu + K_gps*(y_gps(:,i)-C_gps*x_gps); %4 XHAT
    x_imu(1:2) = x_gps(1:2)
    GPS_DATA_FLAG = 0;
    disp(i-1)
    else
    x_imu = x_imu+K_imu*(y_imu(:,i)-C_imu*x_imu); %4 XHAT
    end
    
    Pk_imu = (eye(size(K_imu,1)) - K_imu*C_imu)*Pk_imu; %5
    X_imu = [X_imu x_imu];

    
    
    
    
    i = i+1;
    
    
    end
    
      SYS_gps = ss(A_gps,[B_gps eye(6)],C_gps,0,Ts_gps);
      [KEST_gps,K_KALM_gps,P_gps] = kalman(SYS_gps,Rw_gps,Rv_gps);
    
    plot(t,X_imu)
    figure
    plot(t_gps,X_gps)
%     plot(t_gps,X_gps(1:2,:),'*')



