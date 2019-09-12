clear;close all;clc

load('GPSData.txt');

latlong = -GPSData(:,1:2);
dlmwrite('latlong.txt',latlong,'precision','%.7f')

X = GPSData(:,5);
Y = GPSData(:,6);

plot(X,Y)
grid on;
distance = sqrt(X(end)^2 + Y(end)^2);

% Velocity
x1 = X;
x2 = [0; x1(1:end-1)];
vx = x1-x2/1; %Ts = 1


y1 = Y;
y2 = [0; y1(1:end-1)];
vy = y1-y2/1; %Ts = 1

%filtra descontinuidades

for i=1:length(vx)
    if vx(i) > 30 || vx(i) < -30
       vx(i) = vx(i-1) ;
    end
    if vy(i) > 30 || vy(i) < -30
        vy(i) = vy(i-1) ;
    end
end

V = sqrt(vx.^2 + vy.^2);

t = 1:length(vx);
figure
plot(t,vy*3.6)
hold on
plot(t,vx*3.6)
grid on;

figure
plot(t,V*3.6)
grid on;
hold on;
legend('Speed KM/h')
xlabel('Time [s]')