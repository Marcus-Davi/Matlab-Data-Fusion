%Aqui levamos em conta o bias do gyro
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
    sizes.NumDiscStates  = 4+3;
    sizes.NumOutputs     = 4+3;
    sizes.NumInputs      = 3+3+3; %wmeasure + [acc mag]
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = zeros(sizes.NumDiscStates,1) + [1 0 0 0 0 0 0]';
    str = [ ];
    ts = Ts; %tempo de amostragem vari�vel
    
    Pk = zeros(sizes.NumDiscStates);
    
    
  
elseif flag == 2 %State Update
q_k = x(1:4); %quart rotate
bias_k = x(5:end); %bias
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


%PREDICT
qk_est = (q_k' + (Ts/2)*quatmultiply(q_k',([0 w_measure'] - [0 bias_k'])))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 
bias_est = bias_k;

a_est = quatrotate(qk_est',g')'; %nav2body

m_est = quatrotate(qk_est',m')'; %nav2body

y_est = [a_est;m_est]; % 6x1

% Jf = (Ts/2)*[2/Ts -w_measure(1) -w_measure(2) -w_measure(3);
%       w_measure(1) 2/Ts w_measure(3) -w_measure(2);
%       w_measure(2) -w_measure(3) 2/Ts w_measure(1)
%       w_measure(3) w_measure(2) -w_measure(1) 2/Ts];

Jf = (Ts/2)*[2/Ts -(w_measure(1)-bias_k(1)) -(w_measure(2)-bias_k(2)) -(w_measure(3)-bias_k(3)) q_k(2) q_k(3) q_k(4);
      (w_measure(1)-bias_k(1)) 2/Ts (w_measure(3)-bias_k(3)) -(w_measure(2)-bias_k(2)) -q_k(1) q_k(4) -q_k(3);
      (w_measure(2)-bias_k(2)) -(w_measure(3)-bias_k(3)) 2/Ts (w_measure(1)-bias_k(1)) -q_k(4) -q_k(1) -q_k(2);
      (w_measure(3)-bias_k(3)) (w_measure(2)-bias_k(2)) -(w_measure(1)-bias_k(1)) 2/Ts q_k(3) -q_k(2) -q_k(1);
      0 0 0 0 2/Ts 0 0;
      0 0 0 0 0 2/Ts 0;
      0 0 0 0 0 0 2/Ts];




Jh1 =2*9.8*[-qk_est(3) qk_est(4) -qk_est(1) qk_est(2) 0 0 0;
    qk_est(2) qk_est(1) qk_est(4) qk_est(3) 0 0 0;
    qk_est(1) -qk_est(2) -qk_est(3) qk_est(4) 0 0 0];


Jh2 = 2*mag_m*[qk_est(1) qk_est(2) -qk_est(3) -qk_est(4) 0 0 0;
    -qk_est(4) qk_est(3) qk_est(2) -qk_est(1) 0 0 0;
    qk_est(3) qk_est(4) qk_est(1) qk_est(2) 0 0 0];
           
Jh2 = Jh2 + 2*mag_n*[-qk_est(3) qk_est(4) -qk_est(1) qk_est(2) 0 0 0;
    qk_est(2) qk_est(1) qk_est(4) qk_est(3) 0 0 0;
    qk_est(1) -qk_est(2) -qk_est(3) qk_est(4) 0 0 0];

    
    Jh = [Jh1;Jh2];
    
    %PREDICT
    xhat = [qk_est; bias_est];
    Pk = Jf*Pk*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*Jh'/(Jh*Pk*Jh'+Rn);
    sys = xhat + Kk*(yin-y_est);
    Pk = (eye(length(Pk)) - Kk*Jh)*Pk;
    
    
%     Linear
%     xhat = A*x + B*uin;   %1
%     P = A*P*A' + Rw; %2 
%     
%     K = (P*C')/(C*P*C' + Rv); %3
%     sys = xhat+K*(yin-C*xhat); %4
%     P = (eye(size(K,1)) - K*C)*P; %5
   

elseif flag == 3 %Output

    sys = x;
    q_k = x(1:4);
%     b_k = x(5:end);
    v = [1 0 0];
    r = quatrotate(quatconj(q_k'),v);
   
%     subplot(2,1,1)
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
