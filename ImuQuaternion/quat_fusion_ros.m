%% Estabelecimento da Conexao Bluetooth
if exist('porta')
    fclose(porta);
end
clear ;close all;clc
% rosinit
disp('Conectando a Porta Serial...');
global porta
% porta = serial('/dev/ttyACM0','BaudRate',115200,'Terminator','CR');
porta = serial('/dev/ttyACM0','BaudRate',115200,'Terminator','CR','Timeout',10);
fopen(porta);
disp('Conectado.');

Ts = 1/50;
Qn = [ 0.001, -0.0003, 0.0003, 0.0003,
    		-0.0003,0.0001,-0.0001,-0.0001,
    		0.0003,-0.0001,0.0001,0.0001,
    		0.0003,-0.0001,0.0001,0.0001];

Rn = 0.1*eye(3);
% Rn = 0.71e-3*eye(6); 
% Rn(4,4) = 2;
% Rn(5,5) = 2;
% Rn(6,6) = 3;
% return



% sim('simula')
% return
q_k = [1 0 0 0]';
g = [0 0 9.8]';
Pk = eye(4);

%%  ROS

tftree = rostf;

tform_m = rosmessage('geometry_msgs/TransformStamped');
tform_m.ChildFrameId = 'matlab';
tform_m.Header.FrameId = 'map';
tform_m.Transform.Translation.X = 0.5;


tform_p = rosmessage('geometry_msgs/TransformStamped');
tform_p.ChildFrameId = 'k64f';
tform_p.Header.FrameId = 'map';
tform_p.Transform.Translation.X = 0.25;



%% APENAS PLOTA O QUATERNION DA PLACA
flushinput(porta);
while(1)
    
    while(porta.BytesAvailable==0)
    end
    
%     x = fscanf(porta,'%f %f %f %f %f %f\n\r')
    x = fscanf(porta,'%f %f %f %f %f %f %f %f %f %f\n\r')

% m_inclination = deg2rad(19.2568); %graus (ADQUIRIDO ONLINE)
w_measure = x(1:3);
a_measure = x(4:6);
q_placa = x(7:end);
yin = a_measure;

%x_k+1 = x_k + f(x_k)dt + G.udt

q_k = (q_k' + (Ts/2)*quatmultiply(q_k',[0; w_measure]'))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 
x_est = q_k;
%bias_k1 = bias' - Ts*[0.1 0 0;0 0.1 0;0 0 0.1]*bias' + Ts*bias_adj';

% y_q = quatmultiply(quatconj(q_k1'),[0 0 0 9.8]);%y = h(x)
% y_est = quatmultiply(y_q,q_k1')';
a_est = quatrotate(x_est',g')'; %nav2body

% m_est = quatrotate(x_est',m')'; %nav2body

% y_est = [a_est;m_est]; % 6x1
y_est = [a_est]; % 3x1

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

% Jh2 = 2*mag_m*[x_est(1) x_est(2) -x_est(3) -x_est(4);
%     -x_est(4) x_est(3) x_est(2) -x_est(1);
%     x_est(3) x_est(4) x_est(1) x_est(2)];
%            
% Jh2 = Jh2 + 2*mag_n*[-x_est(3) x_est(4) -x_est(1) x_est(2);
%     x_est(2) x_est(1) x_est(4) x_est(3);
%     x_est(1) -x_est(2) -x_est(3) x_est(4)];


%     Jh2 = zeros(4);
    
%     Jh = [Jh1;Jh2];
    Jh = [Jh1];
    
    %PREDICT
    xhat = q_k;
    Pk = Jf*Pk*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*Jh'/(Jh*Pk*Jh'+Rn);
    q_k = xhat + Kk*(yin-y_est)
    Pk = (eye(4) - Kk*Jh)*Pk;
    
    

    
%     v = [1 0 0];
%     r = quatrotate(quatconj(q_k'),v);
% %     r_placa = quatrotate(quatconj(q_placa'),v);
%     plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
%     plot3([0 0],[0 1],[0 0],'black','linewidth',2);
%     plot3([0 0],[0 0],[0 1],'black','linewidth',2);
%     plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
% %     plot3([0 r_placa(1)],[0 r_placa(2)],[0 r_placa(3)],'red','linewidth',2);
%     xlim([-1 1]);
%     ylim([-1 1]);
%     zlim([-1 1]);
%     hold off;
%     drawnow
q_send = quatnormalize(q_k')
tform_m.Transform.Rotation.W = q_send(1);
tform_m.Transform.Rotation.X = q_send(2);
tform_m.Transform.Rotation.Y = q_send(3);
tform_m.Transform.Rotation.Z = q_send(4);

q_send_placa = quatnormalize(q_placa')
tform_p.Transform.Rotation.W = q_send_placa(1);
tform_p.Transform.Rotation.X = q_send_placa(2);
tform_p.Transform.Rotation.Y = q_send_placa(3);
tform_p.Transform.Rotation.Z = q_send_placa(4);
    
    sendTransform(tftree,tform_m)
    sendTransform(tftree,tform_p)



end
    
    

% Pitch
