clear;close all;clc
Freq = 5;
Lab0 = [-3.743718 -38.577979 0];
GPS = gpsSensor('UpdateRate',Freq,'ReferenceLocation',Lab0,'HorizontalPositionAccuracy',3,'DecayFactor',0.5);

Samples = 1000;
trueX = 0*ones(Samples,3);
trueV = 0*ones(Samples,3);

[p,v,gs,c] = GPS(trueX,trueV);


% [x,y] = utm(p(:,1),p(:,2));
% [lx,ly] = utm(Lab0(1),Lab0(2));

% return

[x_eq,y_eq] = equiret(p(:,1),p(:,2),Lab0(1),Lab0(2));

fprintf('std_dev x = %f\n',sqrt(var(x_eq)))
fprintf('std_dev vx = %f\n',sqrt(var(v(:,1))))


plot(x_eq,y_eq,'*')
% hold on
% plot(x-lx,y-ly,'.')
grid on
figure
plot(v(:,1))

