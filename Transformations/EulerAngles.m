clear ; close all; clc
syms psi theta phi %phi: yaw | theta : roll | psi : pitch
% psi = 45*pi/180;
%Body2Nav := rotação em torno de um ponto
R_psi = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)]
R_theta = [cos(theta) 0  sin(theta);0 1 0;-sin(theta) 0 cos(theta)]
R_phi = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1] %rotação regra-mao-direita

%BODY 2 NAV! %Matlab precedencia comeca pela direita
R_B2N = R_psi*R_theta*R_phi %Modelo ZYX 
R_N2B = R_psi'*R_theta'*R_phi' %Modelo ZYX

Acc = R_psi'*R_theta'


x0 = [1 0 0]';
y0 = [0 1 0]';
z0 = [0 0 1]';
%return


v0 = [1 1 0]';
v1 = R_psi*v0;
plot3([0 x0(1)],[0 x0(2)],[0 x0(3)],'black','linewidth',2); hold on; grid on;
plot3([0 y0(1)],[0 y0(2)],[0 y0(3)],'black','linewidth',2);
plot3([0 z0(1)],[0 z0(2)],[0 z0(3)],'black','linewidth',2);

plot3([0 v0(1)],[0 v0(2)],[0 v0(3)],'red','linewidth',2); 
plot3([0 v1(1)],[0 v1(2)],[0 v1(3)],'blue','linewidth',2); 

figure
x1 = R_psi*x0;
y1 = R_psi*y0;
z1 = R_psi*z0;

%Gira quadro sentido regra-mao-direita
plot3([0 x1(1)],[0 x1(2)],[0 x1(3)],'black','linewidth',2); hold on; grid on;
plot3([0 y1(1)],[0 y1(2)],[0 y1(3)],'black','linewidth',2);
plot3([0 z1(1)],[0 z1(2)],[0 z1(3)],'black','linewidth',2);

plot3([0 v0(1)],[0 v0(2)],[0 v0(3)],'red','linewidth',2); 
disp('vetor original quadro 0 ')
disp(v0);
disp('vetor rotacionado quadro 0')
disp(R_psi*v0);
disp('vetor original quadro 1')
disp(R_psi'*v0);






