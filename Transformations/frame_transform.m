clear;close all;clc
altura_quadro = 1;
T = [1; 1; altura_quadro; 1];
psi = 0; % roll
theta = 0; %pitch
phi= pi/4;  %yaw
R_psi = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)];
R_theta = [cos(theta) 0  sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
R_phi = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1]; %rotação regra-mao-direita

R_B2N = R_psi*R_theta*R_phi;%Modelo ZYX
R_N2B = R_psi'*R_theta'*R_phi'; %Modelo ZYX

R = R_B2N;
% R = R_N2B;
Tr = [[R;[0 0 0]] T];

% Tr é a transformação afim que leva o "frame 0" ao "frame 1"

%% Frame Transforms
p0 = [3 0 0]'; % ponto de prova

p1 = Tr*[p0;1]; % supor que o ponto de prova p0 esteja
% com coordenadas [3 0 0]' relativas ao seu próprio quadro, "frame 1". sendo assim p1
% = Tr*p0  são as coordenadas de p0 em relação ao "frame 0".

p1_ = inv(Tr)*[p0;1]; % supor que o ponto de prova p0 esteja
% com coordenadas [3 0 0]' relativas ao seu próprio quadro, "frame 0". sendo assim p1
% = inv(Tr)*p0  são as coordenadas de p0 em relação ao "frame 1".

% Sendo assim, entende-se que um ponto "p", referente ao "quadro 1", pode ser
% descrito em termos do "quadro 1" bastando transformar/multiplicaor o ponto "p"
% pela mesma matriz que leva o "quadro 0" ao "quadro 1"

% Plots
O = [0 0 0]'; %origin
x0 = [1 0 0]';
y0 = [0 1 0]';
z0 = [0 0 1]';
%return


plot3([O(1) x0(1)],[O(2) x0(2)],[O(3) x0(3)],'red','linewidth',2); hold on; grid on;
plot3([O(1) y0(1)],[O(2) y0(2)],[O(3) y0(3)],'blue','linewidth',2);
plot3([O(1) z0(1)],[O(2) z0(2)],[O(3) z0(3)],'green','linewidth',2);

% new "origin"
NO = Tr*[O;1]; %transform origin
% new "Unit vectors"
x1 = Tr*[x0;1]; %transform unit vectors
y1 = Tr*[y0;1];
z1 = Tr*[z0;1];
plot3([NO(1) x1(1)],[NO(2) x1(2)],[NO(3) x1(3)],'red','linewidth',2); hold on; grid on;
plot3([NO(1) y1(1)],[NO(2) y1(2)],[NO(3) y1(3)],'blue','linewidth',2);
plot3([NO(1) z1(1)],[NO(2) z1(2)],[NO(3) z1(3)],'green','linewidth',2);

% Plots Points
plot3(p0(1),p0(2),p0(3),'*')
plot3(p1(1),p1(2),p1(3),'*')


scale = 6;
xlim([-scale scale])
ylim([-scale scale])
zlim([-scale scale])



