function [sys, x0, str,ts] = sfunc(t,x,u,flag,Ts,Qn,Rn)
persistent Pk;
persistent Kk;
%t = tempo
%x = estados
%u = entrada
%flag = indicador de task

%sys = saida
%x0 = condicao inciial
%str = vazio
%ts = tempo de amostragem

if flag == 0 %Inicializacao

    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 6; %biases and angles
    sizes.NumOutputs     = 6;
    sizes.NumInputs      = 3+3+3; %wmeasure + [acc mag]
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = zeros(sizes.NumDiscStates,1);
    str = [ ];
    ts = Ts; %tempo de amostragem vari�vel
    
    Pk = zeros(sizes.NumDiscStates);
    
    
  
elseif flag == 2 %State Update

m_inclination = deg2rad(19.2568); %graus (ADQUIRIDO ONLINE)
a_measure = [-u(2) u(1) u(3)]' * 0.488e-3 * 9.80665; %m/s²
w_measure = [-u(5) u(4) u(6)]' * 15.625e-3 * pi/180; %rad / s
m_measure = [-u(8) u(7) u(9)]' * 0.1;  %uT

m_field = norm(m_measure);
g = [0 0 9.8]';
mag_m  = m_field*cos(m_inclination);
mag_n = m_field*sin(m_inclination);
m = [mag_m 0 mag_n]';


yin = [atan2(a_measure(2),a_measure(3)); %roll
       atan2(-a_measure(1),sqrt(a_measure(2)^2+a_measure(3)^2)); %pitch
       atan2(-m_measure(2),m_measure(1))]; %yaw

A = [1 0 0 -Ts 0 0;
    0 1 0 0 -Ts 0;
    0 0 1 0 0 -Ts;
    0 0 0 1 0 0 ;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

B = [Ts 0 0;0 Ts 0;0 0 Ts;0 0 0;0 0 0;0 0 0];

C = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0];





%x_k+1 = x_k + f(x_k)dt + G.udt


    
    %PREDICT
    xhat = A*x + B*w_measure;
    Pk = A*Pk*A'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*C'/(C*Pk*C'+Rn);
    sys = xhat + Kk*(yin-C*xhat);
    Pk = (eye(6) - Kk*C)*Pk;
    
    
%     Linear
%     xhat = A*x + B*uin;   %1
%     P = A*P*A' + Rw; %2 
%     
%     K = (P*C')/(C*P*C' + Rv); %3
%     sys = xhat+K*(yin-C*xhat); %4
%     P = (eye(size(K,1)) - K*C)*P; %5
   

elseif flag == 3 %Output

    sys = x;
    
    psi = x(1)
    theta = x(2)
    phi = x(3)
    
    v = [1 0 0]';
    
R_psi = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)];
R_theta = [cos(theta) 0  sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
R_phi = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1]; %rotação regra-mao-direita

%BODY 2 NAV! %Matlab precedencia comeca pela direita
R_B2N = R_psi*R_theta*R_phi; %Modelo ZYX 
r = R_B2N*v;

    plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
    plot3([0 0],[0 1],[0 0],'black','linewidth',2);
    plot3([0 0],[0 0],[0 1],'black','linewidth',2);
    plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    hold off;

%     subplot(2,1,2)
%     plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
%     plot3([0 0],[0 1],[0 0],'black','linewidth',2);
%     plot3([0 0],[0 0],[0 1],'black','linewidth',2);
%     plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
%     xlim([-1 1]);
%     ylim([-1 1]);
%     zlim([-1 1]);
%     hold off
%    view(0,90);
    
%         subplot(3,1,3)
%     plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
%     plot3([0 0],[0 1],[0 0],'black','linewidth',2);
%     plot3([0 0],[0 0],[0 1],'black','linewidth',2);
%     plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
%     xlim([-1 1]);
%     ylim([-1 1]);
%     zlim([-1 1]);
%     hold off
%     view(0,0)
%     
    
    drawnow;

elseif flag == 9
    sys = [ ]; %n�o faz nada
end
