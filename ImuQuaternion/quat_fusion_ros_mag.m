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

Rn_ga = 0.1*eye(3);
Rn = 0.1*eye(6); 
Rn(4,4) = 2;
Rn(5,5) = 2;
Rn(6,6) = 3;
% return



% sim('simula')
% return
m_inclination = deg2rad(19.2568); %graus (ADQUIRIDO ONLINE)
q_k = [1 0 0 0]';
q_k_gyro = q_k;
q_k_gyro_accel = q_k;
g = [0 0 9.8]';
mn_ = [cos(m_inclination) 0 sin(m_inclination)]';
Pk = zeros(4);

Pk_ga = eye(4);

%%  ROS

tftree = rostf;

tform_m = rosmessage('geometry_msgs/TransformStamped');
tform_m.ChildFrameId = 'total_fusion';
tform_m.Header.FrameId = 'map';
tform_m.Transform.Translation.X = 1.2;


tform_ga = rosmessage('geometry_msgs/TransformStamped');
tform_ga.ChildFrameId = 'gyro_accel';
tform_ga.Header.FrameId = 'map';
tform_ga.Transform.Translation.X = 0.8;

tform_g = rosmessage('geometry_msgs/TransformStamped');
tform_g.ChildFrameId = 'gyro';
tform_g.Header.FrameId = 'map';
tform_g.Transform.Translation.X = 0.4;





%% APENAS PLOTA O QUATERNION DA PLACA
flushinput(porta);
while(1)
    
    while(porta.BytesAvailable==0)
    end
    
%     x = fscanf(porta,'%f %f %f %f %f %f\n\r')
    x = fscanf(porta,'%f %f %f %f %f %f %f %f %f\n\r')


w_measure = x(1:3);
a_measure = x(4:6);
m_measure = x(7:9);

mag_field = sqrt(m_measure'*m_measure);

yin = [a_measure;m_measure];
yin_ga = a_measure;

%x_k+1 = x_k + f(x_k)dt + G.udt
q_k = (q_k' + (Ts/2)*quatmultiply(q_k',[0; w_measure]'))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 
%bias_k1 = bias' - Ts*[0.1 0 0;0 0.1 0;0 0 0.1]*bias' + Ts*bias_adj';

q_k_gyro = (q_k_gyro' + (Ts/2)*quatmultiply(q_k_gyro',[0; w_measure]'))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 

q_k_gyro_accel = (q_k_gyro_accel' + (Ts/2)*quatmultiply(q_k_gyro_accel',[0; w_measure]'))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]); 

mn = mn_*mag_field;

a_est = quatrotate(q_k',g')'; %nav2body
m_est = quatrotate(q_k',mn')'; %nav2body

a_est_ga = quatrotate(q_k_gyro_accel',g')'; %nav2body

y_est = [a_est;m_est]; % 6x1

y_est_ga =  a_est_ga;
% y_est = [a_est]; % 3x1


Jf = (Ts/2)*[2/Ts -w_measure(1) -w_measure(2) -w_measure(3);
      w_measure(1) 2/Ts w_measure(3) -w_measure(2);
      w_measure(2) -w_measure(3) 2/Ts w_measure(1)
      w_measure(3) w_measure(2) -w_measure(1) 2/Ts];
  
%  x_est = x;

Jh1 =2*9.8*[-q_k(3) q_k(4) -q_k(1) q_k(2);
    q_k(2) q_k(1) q_k(4) q_k(3);
    q_k(1) -q_k(2) -q_k(3) q_k(4)];

Jh2 = 2*mn(1)*[q_k(1) q_k(2) -q_k(3) -q_k(4);
    -q_k(4) q_k(3) q_k(2) -q_k(1);
    q_k(3) q_k(4) q_k(1) q_k(2)];
           
Jh2 = Jh2 + ...
    2*mn(3)*[-q_k(3) q_k(4) -q_k(1) q_k(2);
    q_k(2) q_k(1) q_k(4) q_k(3);
    q_k(1) -q_k(2) -q_k(3) q_k(4)];


Jh_ga =2*9.8*[-q_k_gyro_accel(3) q_k_gyro_accel(4) -q_k_gyro_accel(1) q_k_gyro_accel(2);
    q_k_gyro_accel(2) q_k_gyro_accel(1) q_k_gyro_accel(4) q_k_gyro_accel(3);
    q_k_gyro_accel(1) -q_k_gyro_accel(2) -q_k_gyro_accel(3) q_k_gyro_accel(4)];


%     Jh2 = zeros(4);
    
    Jh = [Jh1;Jh2];
%     Jh = [Jh1];
    
    %PREDICT
%     xhat = q_k;
    Pk = Jf*Pk*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*Jh'/(Jh*Pk*Jh'+Rn);
    q_k = q_k + Kk*(yin-y_est);
    Pk = (eye(4) - Kk*Jh)*Pk;
    
        %PREDICT
%     xhat = q_k;
    Pk_ga = Jf*Pk_ga*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk_ga*Jh_ga'/(Jh_ga*Pk_ga*Jh_ga'+Rn_ga);
    q_k_gyro_accel = q_k_gyro_accel + Kk*(yin_ga-y_est_ga);
    Pk = (eye(4) - Kk*Jh_ga)*Pk;
    

q_gyro_norm = quatnormalize(q_k_gyro')
tform_g.Transform.Rotation.W = q_gyro_norm(1);
tform_g.Transform.Rotation.X = q_gyro_norm(2);
tform_g.Transform.Rotation.Y = q_gyro_norm(3);
tform_g.Transform.Rotation.Z = q_gyro_norm(4);

q_gyro_accel_norm = quatnormalize(q_k_gyro_accel')
tform_ga.Transform.Rotation.W = q_gyro_accel_norm(1);
tform_ga.Transform.Rotation.X = q_gyro_accel_norm(2);
tform_ga.Transform.Rotation.Y = q_gyro_accel_norm(3);
tform_ga.Transform.Rotation.Z = q_gyro_accel_norm(4);

q_total = quatnormalize(q_k')
tform_m.Transform.Rotation.W = q_total(1);
tform_m.Transform.Rotation.X = q_total(2);
tform_m.Transform.Rotation.Y = q_total(3);
tform_m.Transform.Rotation.Z = q_total(4);
    
    sendTransform(tftree,tform_m)
    sendTransform(tftree,tform_g)
    sendTransform(tftree,tform_ga)



end
    
    

% Pitch
