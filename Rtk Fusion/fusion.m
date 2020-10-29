clear;close all;clc
load('mission_experiment.mat');


time_rtk = rtk_data(:,1);
time_gps = gps_data(:,1);
time_correspondences = zeros(length(time_rtk),1);
for i=1:length(time_rtk)
    idx = getClosest(time_rtk(i),time_gps);
    time_correspondences(i) = idx; % Kalman should obey this
    
end


%% Fusion
Ts = 1/50;
%Model 1
% x = [0 0]'; %[ x, y ]
% A = eye(2);
% B = [Ts Ts^2/2 0 0;0 0 Ts Ts^2/2];
% C = eye(2);
% D = 0;
% P = eye(length(x));
% Qk = 5*eye(2); %VELS
% Rk = 10*eye(2); % RTK

% Model 2
x = [0 0 0 0]';
A = [1 Ts 0 0;0 1 0 0;0 0 1 Ts;0 0 0 1];
B =  [Ts^2/2 0;Ts 0;0 Ts^2/2;0 Ts];
Crtk = [1 0 0 0;0 0 1 0];
Cvel = [0 1 0 0;0 0 0 1];
D = 0;
P = eye(length(x));
% Q = 100*[Ts^3^3 Ts^2/2;Ts^2/2 Ts];
Q = eye(2);
Qk = blkdiag(Q,Q); %Acc
% Qk = 0.03*eye(4);
Rrtk = 0.01*eye(2);
Rvel = 0.1*eye(2);


idx_vel_old = 0;
idx_rtk_old = 0;
N = length(Accs);
fused = zeros(N,length(x));
x = [0 0 0 0]';

for i=2:N
    % Synchronize
    idx_vel = getClosest(Accs(i,1),Vels(:,1));
    idx_gps = getClosest(Accs(i,1),gps_data(:,1));
    idx_rtk = getClosest(Accs(i,1),rtk_data(:,1));
    
    
    u = [Accs(i,2) Accs(i,3)]'; %Ax Ay
    
    %Predict (with accelerations)
    %Reform equations
    Ts = Accs(i,1) - Accs(i-1,1);
    A = [1 Ts 0 0;0 1 0 0;0 0 1 Ts;0 0 0 1];
    B =  [Ts^2/2 0;Ts 0;0 Ts^2/2;0 Ts];
    Crtk = [1 0 0 0;0 0 1 0];
    Cvel = [0 1 0 0;0 0 0 1];
    D = 0;
    P = eye(length(x));
    Q = [Ts^3^3 Ts^2/2;Ts^2/2 Ts];
    Qk = 0.03*blkdiag(Q,Q); %Acc
    
    x = A*x + B*u;
    P = A*P*A' + Qk;
    
    %new data
    % if(idx_gps ~= idx_old)
    % y_rtk = [gps_data(idx_gps,2) gps_data(idx_gps,3)]';
    
    if(idx_vel ~= idx_vel_old)
        y_vel = [Vels(idx_vel,2)  Vels(idx_vel,3)]';
        err = y_vel - Cvel*x;
        
        Rvel = (0.01/(Ts))*eye(2);
        
        S = Cvel*P*Cvel' + Rvel;
        K = P*Cvel'*inv(S);
        
        x = x + K*err;
        
        P = (eye(length(x))-K*Cvel)*P;
        
    end
    
    if(idx_rtk ~= idx_rtk_old)
        y_rtk = [rtk_data(idx_rtk,2) rtk_data(idx_rtk,3)]';
        err = y_rtk - Crtk*x;
        
        Rrtk = (0.5/Ts)*eye(2);
        
        S = Crtk*P*Crtk' + Rrtk;
        K = P*Crtk'*inv(S);
        
        x = x + K*err; %final x
        
        P = (eye(length(x))-K*Crtk)*P;
        
    end
    
    
    idx_rtk_old = idx_rtk;
    idx_vel_old = idx_vel;
    
    fused(i,:) = x';
    
    
%     plot(rtk_data(1:idx_rtk,2),rtk_data(1:idx_rtk,3),'*')
%     hold on
%     plot(gps_data(1:idx_gps,2),gps_data(1:idx_gps,3))
%     plot(fused(1:i,1),fused(1:i,3))
%     hold off
%     % pause(0.01)
%     drawnow
%     
    
    
end

plot(rtk_data(1:idx_rtk,2),rtk_data(1:idx_rtk,3),'*')
hold on
plot(gps_data(1:idx_gps,2),gps_data(1:idx_gps,3),'linewidth',2)
plot(fused(1:i,1),fused(1:i,3),'r','linewidth',2)
hold off
% pause(0.01)
drawnow
legend('Pure RTK','DJI Fusion','Marcus Fusion')
grid on

% return


%% Utilities


% Assume sorted vector
% function res = isSync(index,sorted_indexes)
% n = length(sorted_indexes);
% for i=1:n
%     if(index == sorted_indexes(i))
%         res = true;
%         return
%     end
%     
% end
% 
% res = false;
% 
% end

function index = getClosest(value,vector)
diff = value - vector;
[~,index] = min(abs(diff));

end