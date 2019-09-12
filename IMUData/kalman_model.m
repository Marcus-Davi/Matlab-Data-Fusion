clear;close all;clc
%% Sensor Characteristics GY-80
Ts = 1/100; %100Hz , Sensor Sampling Time
load('sensor_data.mat')
g = 9.8;
% plot(y)
Acc.noise_lsb = 0.75;
Acc.res = 4e-3; % mg/lsb
Gyr.res = 17.5e-3; %deg/lsb
Gyr.noise_density = 0.03; %dps/sqrt(Hz)
Mag.noise_ = 4.35; %mili-gauss
% Experimental
Acc.data.x = y(:,1);
Acc.data.y = y(:,2);
Acc.data.z = y(:,3);


Gyr.data.x = y(:,4);
Gyr.data.y = y(:,5);
Gyr.data.z = y(:,6);

Mag.data.x = y(:,7);
Mag.data.y = y(:,8);
Mag.data.z = y(:,9);

Acc.noise = var([Acc.data.x Acc.data.y Acc.data.z]); %raw


Acc.pitch =  atan2(-Acc.data.x,sqrt(Acc.data.y.^2 + Acc.data.z.^2));
Acc.roll = atan2(Acc.data.y,Acc.data.z);

Acc.noise_angle = var([Acc.pitch Acc.roll]);

Gyr.noise = var(deg2rad([Gyr.data.x Gyr.data.y Gyr.data.z])*Gyr.res); %converted


Mag.yaw = atan2(-Mag.data.y,Mag.data.x);

Mag.noise = var(Mag.yaw);

% Acc.noise = [115.9364  179.9914   99.4625]; %X Y Z aceleracao
% Acc.noise_angle = [0.0015 0.0020]; %pitch, roll
% 
% Gyr.noise = [74.5687  114.4020  198.6273]; %roll pitch yaw
% 
% Mag.noise = [2.6214  2.5360 2.3241];

%% Kalman Models

A = [1 -Ts;0 1];
B = [Ts;0];
C = [1 0];
D = 0;
sys = ss(A,[B eye(2)],C,0,Ts);
Q_bias = Ts^2/2;
% Pitch

Q = [Gyr.noise(2) 0;0 Q_bias]; %process
% Q = 0.01*[Ts^2/2 0;0 Ts]
R = Acc.noise_angle(2); %measure

[KEST,L_pitch,P] = kalman(sys,Q,R);
L_pitch
% Roll
Q = [Gyr.noise(1) 0;0 Q_bias]; %process
R = Acc.noise_angle(2); %measure

[KEST,L_roll,P] = kalman(sys,Q,R);
L_roll
%Yaw
Q = [Gyr.noise(3) 0;0 Q_bias]; %process
R = Mag.noise; %measure
[KEST,L_yaw,P] = kalman(sys,Q,R);
L_yaw

