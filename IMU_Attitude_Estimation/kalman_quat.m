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
    sizes.NumDiscStates  = 4;
    sizes.NumOutputs     = 4;
    sizes.NumInputs      = 3+3+3; %wmeasure + [acc mag]
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = zeros(sizes.NumDiscStates,1) + [1 0 0 0]';
    str = [ ];
    ts = Ts; %tempo de amostragem vari�vel
    
    Pk = zeros(size(4));
    
    
  
elseif flag == 2 %State Update
q_k = x; %quart rotate
m_inclination = deg2rad(19.2568); %graus (ADQUIRIDO ONLINE)
a_measure = [-u(2) u(1) u(3)]' * 0.488e-3 * 9.80665; %m/s²
w_measure = [-u(5) u(4) u(6)]' * 15.625e-3 * pi/180; %rad / s
m_measure = [-u(8) u(7) u(9)]' * 0.1;  %uT

% a_measure = [u(1) u(2) u(3)]' * 0.488e-3 * 9.80665; %m/s²
% w_measure = [u(4) u(5) u(6)]' * 15.625e-3 * pi/180; %rad / s
% m_measure = [u(7) u(8) u(9)]' * 0.1  %uT


m_field = norm(m_measure);
g = [0 0 9.8]';
mag_m  = m_field*cos(m_inclination);
mag_n = m_field*sin(m_inclination);
m = [mag_m 0 mag_n]';
% yin = a_measure;
yin =[a_measure;m_measure];%

%x_k+1 = x_k + f(x_k)dt + G.udt

q_k1 = (q_k' + (Ts/2)*quatmultiply(q_k',[0 w_measure']))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 
x_est = q_k1;
%bias_k1 = bias' - Ts*[0.1 0 0;0 0.1 0;0 0 0.1]*bias' + Ts*bias_adj';

% y_q = quatmultiply(quatconj(q_k1'),[0 0 0 9.8]);%y = h(x)
% y_est = quatmultiply(y_q,q_k1')';
a_est = quatrotate(x_est',g')'; %nav2body

m_est = quatrotate(x_est',m')'; %nav2body

y_est = [a_est;m_est]; % 6x1

Jf = (Ts/2)*[2/Ts -w_measure(1) -w_measure(2) -w_measure(3);
      w_measure(1) 2/Ts w_measure(3) -w_measure(2);
      w_measure(2) -w_measure(3) 2/Ts w_measure(1)
      w_measure(3) w_measure(2) -w_measure(1) 2/Ts];
  
%  x_est = x;

Jh1 =2*9.8*[-x_est(3) x_est(4) -x_est(1) x_est(2);
    x_est(2) x_est(1) x_est(4) x_est(3);
    x_est(1) -x_est(2) -x_est(3) x_est(4)];

%REVISAR
% Jh2 = 2*[0 0 0 0;
%       (x(1)*mag_m-x(3)*mag_n) -x(2)*mag_m (-x(3)*mag_m-x(1)*mag_n) -x(4)*mag_m;
%       (x(4)*mag_m+x(2)*mag_n) (x(3)*mag_m+x(1)*mag_n) (x(2)*mag_m+x(4)*mag_n) (x(3)*mag_n+x(1)*mag_m);
%       (x(1)*mag_n+x(3)*mag_m) (-x(2)*mag_n+x(4)*mag_m) (-x(3)*mag_n+x(1)*mag_m) (x(4)*mag_n+x(2)*mag_m)];

Jh2 = 2*mag_m*[x_est(1) x_est(2) -x_est(3) -x_est(4);
    -x_est(4) x_est(3) x_est(2) -x_est(1);
    x_est(3) x_est(4) x_est(1) x_est(2)];
           
Jh2 = Jh2 + 2*mag_n*[-x_est(3) x_est(4) -x_est(1) x_est(2);
    x_est(2) x_est(1) x_est(4) x_est(3);
    x_est(1) -x_est(2) -x_est(3) x_est(4)];


%     Jh2 = zeros(4);
    
    Jh = [Jh1;Jh2];
    
    %PREDICT
    xhat = q_k1;
    Pk = Jf*Pk*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*Jh'/(Jh*Pk*Jh'+Rn);
    sys = xhat + Kk*(yin-y_est);
    Pk = (eye(4) - Kk*Jh)*Pk;
    
    
%     Linear
%     xhat = A*x + B*uin;   %1
%     P = A*P*A' + Rw; %2 
%     
%     K = (P*C')/(C*P*C' + Rv); %3
%     sys = xhat+K*(yin-C*xhat); %4
%     P = (eye(size(K,1)) - K*C)*P; %5
   

elseif flag == 3 %Output

    sys = x;
    v = [1 0 0];
    r = quatrotate(quatconj(x'),v);
   
%     subplot(2,1,1)
%     plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
%     plot3([0 0],[0 1],[0 0],'black','linewidth',2);
%     plot3([0 0],[0 0],[0 1],'black','linewidth',2);
%     plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
%     xlim([-1 1]);
%     ylim([-1 1]);
%     zlim([-1 1]);
%     hold off;

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
