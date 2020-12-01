clear;close all;clc
data = readtable('datax.csv');

ax = data.acc_x;
ay = data.acc_y; % <-
az = data.acc_z;

gx = data.w_roll;
gy = data.w_pitch;
gz = data.w_yall;

 %% Kalman iteratios
 Ts = 1/150;
 A = [1 -Ts;0 1];
 B = [Ts;0];
 C = [1 0];
 Q = 0.1*[Ts^2/2 Ts;Ts^2/2 Ts];
 R = 0.05; %Acc variance
 
 %% Kalman loop
 data_size = length(ax)
 x = [0 0]';
 X = zeros(data_size,2);
 P = 0*ones(length(2));
 for i=1:data_size
    u = gx(i);
    %Predict
    x = A*x + B*u;
    P = A'*P*A + Q;
    
    % Update
    y = atan2(ay(i),az(i)); %sensor read
    
    y_est = C*x;
    error = y - y_est;
    
    S = C*P*C' + R;
    K = P*C'*inv(S);
    
    x = x + K*error;
    P = (eye(2) - K*C)*P;
    
    
    
    X(i,:) = x;
 end
plot(X(:,1))
hold on
plot(atan2(ay,az))
return
%% Plots 
 % Acc
 plot(ax)
 hold on
 plot(ay)
 plot(az)
 
 % Gyr
 figure
 plot(gx)
 hold on
 plot(gy)
 plot(gz)
