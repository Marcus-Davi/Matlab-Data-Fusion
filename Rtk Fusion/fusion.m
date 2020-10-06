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
x = [0 0]';
A = eye(2);
B = [Ts Ts^2/2 0 0;0 0 Ts Ts^2/2];
C = eye(2);
D = 0;
P = eye(2);



Qk = eye(2); %VELS
Rk = 0.05*eye(2); % RTK

new_rtk = false;
idx_old = 0;
N = length(Accs);
fused = zeros(N,2);
for i=1:N
% Synchronize
idx_vel = getClosest(Accs(i,1),Vels(:,1));
idx_gps = getClosest(Accs(i,1),gps_data(:,1));
idx_rtk = getClosest(Accs(i,1),rtk_data(:,1));


u = [Vels(idx_vel,2) Accs(i,2) Vels(idx_vel,3) Accs(i,3)]'; %Ax vx Ay vy
    
%Predict (with accelerations / velocities)
x_ = A*x + B*u;
P = A*P*A' + Qk;

y = C*x_;


%new data
% if(idx_gps ~= idx_old)
% y_rtk = [gps_data(idx_gps,2) gps_data(idx_gps,3)]';

if(idx_rtk ~= idx_old)
y_rtk = [rtk_data(idx_rtk,2) rtk_data(idx_rtk,3)]';
err = y_rtk - y;
S = C*P*C' + Rk;
K = P*C'*inv(S);

x = x_ + K*err;

P = (eye(2)-K*C)*P;

else

x = x_;
    
end


idx_old = idx_rtk;



fused(i,:) = x';






end

% return
%% Plot RTK 2D
plot(rtk_data(:,2),rtk_data(:,3))

hold on
%% PLOT GPS 2D
plot(gps_data(:,2),gps_data(:,3))

plot(fused(:,1),fused(:,2))
legend('RTK','GPS','Fused')

%% Utilities


% Assume sorted vector
function res = isSync(index,sorted_indexes)
    n = length(sorted_indexes);
    for i=1:n
       if(index == sorted_indexes(i))
        res = true;
        return
       end
        
    end
    
    res = false;

end

function index = getClosest(value,vector)
    diff = value - vector;
    [~,index] = min(abs(diff));

end