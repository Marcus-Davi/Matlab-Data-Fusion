clear;close all;clc
Ts_imu = 0.01;
Ts_gps = 1;
A_imu = [1 0 Ts_imu 0;0 1 0 Ts_imu;0 0 1 0;0 0 0 1]

B_imu = [Ts_imu^2/2 0;0 Ts_imu^2/2;Ts_imu 0;0 Ts_imu]

C_imu = [1 0 0 0;0 1 0 0];

% B_imu = zeros(6,1);


Rw_imu = 3*[Ts_imu^4/8 0 Ts_imu^3/3 0;
               0 Ts_imu^4/8 0 Ts_imu^3/3;
               Ts_imu^3/6 0  Ts_imu^2/2 0;
               0 Ts_imu^3/6 0 Ts_imu^2/2];
           
Rw_imu = 0.01*diag([Ts_imu^2/2 Ts_imu^2/2 Ts_imu Ts_imu])

Rv_gps = 1*eye(2); % ruido de potencia alta ( = 1)

    % STEADY STATE KALMAN SOLUTION
    Sys = ss(A_imu,[B_imu eye(4)],C_imu,0,Ts_imu);
    [KEST,K_KALM,P] = kalman(Sys,Rw_imu,Rv_gps);

%% Algoritmo
states = length(A_imu)
t = 0:Ts_imu:100;
t_gps = [];

step_times = t >= 20 & t <= 21;
y_imu = [step_times; step_times]; % ACC ACCELEROMETER READINS
x_imu = zeros(states,1); %INS states

gps_noise = rand(2,length(t))*Rv_gps(1) - Rv_gps(1)/2;
y_gps = 5*[t>=5; t>=5] + gps_noise;


x_imu_nogps = x_imu;

X_imu_nogps = [];
X_imu = [];
X_gps = [];


    P_imu = zeros(states);
    P_imu_nogps = P_imu;
    
    i = 1;
    
    for ta = t
      % PREDICT LOOP RAPIDO
      
     x_imu = A_imu*x_imu + B_imu*y_imu(:,i); %1
     P_imu = A_imu*P_imu*A_imu' + Rw_imu; %2
    
   
    if(rem(i-1,Ts_gps/Ts_imu) == 0) %UPDATE LOOP LENTO
        
    K_imu = (P_imu*C_imu')/(C_imu*P_imu*C_imu' + Rv_gps); %3      
    x_imu = x_imu+K_imu*(y_gps(:,i)-C_imu*x_imu); %4 
    P_imu = (eye(size(K_imu,1)) - K_imu*C_imu)*P_imu; %5
    
%       x_imu = x_imu+K_KALM*(y_gps(:,i)-C_imu*x_imu); %4 

disp(i-1)
    t_gps = [t_gps ta];
    X_gps = [X_gps y_gps(:,i)];
    else
    
    end
    
         % NO GPS
        x_imu_nogps = A_imu*x_imu_nogps + B_imu*y_imu(:,i); %1
        P_imu_nogps = A_imu*P_imu_nogps*A_imu' + Rw_imu; %2
    
    X_imu = [X_imu x_imu];
    X_imu_nogps = [X_imu_nogps x_imu_nogps];
    
    i = i+1;
    
    
    end
    

    
    
    subplot(2,1,1)
    plot(t,X_imu)
    hold on
    plot(t_gps,X_gps,'*')
    grid on
    legend('Fused X','Fused Y','Fused Vx','Fused Vy','GPS X','GPS Y')
    subplot(2,1,2)
    plot(t,y_imu)
    legend('Acceleration')
    grid on
    
    figure
    subplot(2,1,1)
    plot(t,X_imu_nogps)
    grid on;
    legend('IMU X','IMU Y','IMU Vx','IMU Vy')
    subplot(2,1,2)
    plot(t,y_imu)
    legend('Acceleration')


