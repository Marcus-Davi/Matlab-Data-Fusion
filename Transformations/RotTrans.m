clear;close all;clc

%escolha os pontos aqui
% y = [ones(1,10) 3*ones(1,5) ones(1,6) 5*ones(1,7) ones(1,4)];
% x = linspace(1,4,length(y));

x = 0:0.1:pi
y = sin(x)
P0 = [x;y];
figure
hold on
grid on
%% Rotacao em torno da origem

for angle = 0:0.1:2*pi

plot(x,y);

R = [cos(angle) -sin(angle);sin(angle) cos(angle)];

P1 = R*P0;
x1 = P1(1,:);
y1 = P1(2,:);
plot(x1,y1,'b--');

%% Rotação em torno de um ponto arbitrário
p_center = [10;0];

P2 = R*[x-p_center(1);y-p_center(1)] + p_center
x2 = P2(1,:);
y2 = P2(2,:);

plot(x2,y2,'r--');

%% Rotação + Translação
T = [-10;0];
P3 = R*P0  + T;

x_transform = P3(1,:);
y_transform = P3(2,:);

plot(x_transform,y_transform,'g--')
pause(0.1)
end
legend('P_0','P_{rot}','P_{rot2}','P_{trans}')
