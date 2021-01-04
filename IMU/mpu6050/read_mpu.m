clear all;clc; close all
%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: /home/marcus/Workspace/Matlab-Octave/data.csv
%
% Auto-generated by MATLAB on 04-Jan-2021 08:49:11

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 7);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["acc_x", "acc_y", "acc_z", "w_roll", "w_yall", "w_pitch", "time"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable("data.csv", opts);

%% Convert to output type
acc_x = tbl.acc_x;
acc_y = tbl.acc_y;
acc_z = tbl.acc_z;
w_roll = tbl.w_roll;
w_yall = tbl.w_yall;
w_pitch = tbl.w_pitch;
time = tbl.time;

%% Clear temporary variables
clear opts tbl 


%% Parameters
Amostras = length(time);
mean_time = mean(time); % Ts

%% Accelerometer

pitch = atan2(-acc_x,acc_z);

%% Gyro Only

pitch_gyro = zeros(Amostras,1);
for i=2:Amostras
    pitch_gyro(i) = pitch_gyro(i-1) + mean_time*w_pitch(i);
end

%% Kalman
x = [0 0]';
n_estados = 2;
X = zeros(Amostras,n_estados);
% Estados -> Angulo, Time
P = eye(2);
% Covariances | Higher = more uncertainty
Qn = 10000*[mean_time^2/2 0;0 mean_time]; %  Prediction
Rn = 100000; % Correction

for i=1:Amostras
   Ts = mean_time; % deltaT
   
   % Modelo
   A = [1 -Ts;0 1]; 
   B = [Ts;0];
   C = [1 0];
   
   u = w_pitch(i);
   
   % Prediction
   x_ = A*x + B*u;
   P = A*P*A' + Qn;
   
   % Correction
   y_ = C*x; %estimation
   y_measured = pitch(i); %measurement
   err = y_measured - y_; %error
   S = C*P*C' + Rn;
   K = (P*C')*inv(S);
   x = x_ + K*err;
   P = (eye(2) - K*C)*P;
   
   
   
   
%    x = x_;
   X(i,:) = x;
   
end

%% Plots
plot(pitch,'Linewidth',2)
hold on
plot(pitch_gyro,'Linewidth',2)
plot(X(:,1),'Linewidth',2)
grid on
legend('Acc Only','Gyro Only','Kalman')
title('Pitch Motion [rad]')












