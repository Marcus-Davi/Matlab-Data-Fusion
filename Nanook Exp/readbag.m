clear;close all;clc
bag = rosbag('2020-09-21-15-59-07.bag');
GPS = select(bag,'Topic','/gps');
TFs = select(bag,'Topic','/tf')
gps_messages = readMessages(GPS);

%% Plots
lat = gps_messages{1}.Latitude;
lon = gps_messages{1}.Longitude;
[x_0,y_0] = utm(lat,lon);

N = length(gps_messages);
P = zeros(N,2);
LLA = zeros(N,2);
for i=1:length(gps_messages)
    LLA(i,:) = [gps_messages{i}.Latitude, gps_messages{i}.Longitude];
    [utm_x, utm_y] = utm(gps_messages{i}.Latitude, gps_messages{i}.Longitude);
    P(i,:) = [utm_x - x_0, utm_y-y_0];
    % Geoplot
    figure(1)
    geoplot(LLA(1:i,1),LLA(1:i,2),'--o');
    geolimits([-3.744259999999997 -3.743809999999997],[-38.577144999999980 -38.576866666666639]);
    figure(2)
    plot(P(1:i,1),P(1:i,2),'--o')
    drawnow
    i
%     pause(0.1)
    
end

% P(P>100) = 0; % Filtro?

plot(P(:,1),P(:,2))
