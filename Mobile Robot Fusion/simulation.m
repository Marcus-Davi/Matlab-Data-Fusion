clear;close all;clc

Ts = 0.1;
Ts_gps = 0.1;
Tsim = 263;
Samples = round(Tsim/Ts);

%% Gera Trajetória Referencia
p = [0 0 0];  % x,y,theta referenciais para in�cio de trajet�ria.
Xr = p;
i = 0;
load('velocidades.mat')
load('dados_para_fusao.mat')
v_odo = v;
w_odo = w;
for k = 1:Samples  %
    
    
       if(k < Samples/3)
          v(k) = 0.25;
          w(k) = 0;
       elseif(k >= Samples/3 && k<2*Samples/3)
           v(k) = 0.25;
           w(k) = pi/(Samples/3)/Ts; %garante curvatura em U
       else
          v(k) = 0.25;
          w(k) = 0;
       end
       
       p = robot_model(p,[v(k) w(k)],Ts);

   Xr = [Xr; p];

end
vr = v;
wr = w;
figure(1);
plot(Xr(:,1),Xr(:,2),'--black','Linewidth',2)
grid on
% return

%% LQR CONTROLLER DESIGN
ur1 = 1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = 0;
SYS = ss(A,B,C,D);
Q = diag([1 1 0]);
R = 1*eye(2);
[K_LQ,S,E] = lqr(SYS,Q,R);

%% CLOSED LOOP
p = [0 0 0]; %Posição robô
p_odo = [0 0 0];
p_mixed = [0 0 0];p_est_odo = [0 0 0];
P = [];P_noise = [];P_measure = [];P_est = [];
P_odo = [];
U = [];
p_pert = [0 0 0];
p_noise = [0 0 0];
v_pert = [0 0];
p_est = p;

v_lim = 0.5;
w_lim = 0.5;

u1 = 0;
u2 = 0;

variancia_gps = 3; %variancia GPS
variancia_bussola = 0.1; %Variancia BUSSOLA


the_bussola = the_bussola + sqrt(variancia_bussola)*randn(1,Samples+1);

%AJUSTE DO FILTRO DE KALMAN -> MATRIZ COVARIANCIA Q
Q = 0.01*eye(3);
Q(3,3) = 0.0001; %bussola

%VARIANCIA DOS SENSORES (GPS,BUSSOLA)
R = variancia_gps*eye(3);
R(3,3) = variancia_bussola; %bussola

Pk = zeros(3);
for k=1:Samples
    
    % Ruido de medição é inserido aqui  
%     if(mod(k,Ts_gps/Ts) == 0)
%     p_noise = sqrt(variancia_gps)*randn(1,3); %gps
%     p_noise(3) = sqrt(variancia_bussola)*randn(1,1); %bussola
%     end
    
%     p_measure = (p + p_noise); %dados Gerados
    p_measure = [x_gps(k) y_gps(k) the_bussola(k)]; %dados do Bismark
    
    %kalman estendido
    [p_est,~,Pk] = kalman_ext(Ts,@robot_model,@measurement_model,Q,R,p_est,[v_odo(k) w_odo(k)],Pk,p_measure');
    
    p_odo = robot_model(p_odo,[v_odo(k) w_odo(k)],Ts); %odometria pura
    
%     p_est = robot_model(p_est,[u1 u2],Ts)'; 
    
% 
%     e = Xr(k,:) - p_est';
    
%     %Matriz de Rotação (Nav2Body)
%     M = [cos(p(3)) sin(p(3)) 0;
%         -sin(p(3)) cos(p(3)) 0;
%          0 0 1];
%      
%     eb = M*e'; %Transforma erros pro quadro local
%     
%     u = K_LQ*eb; %LQR
%     
%     
%     %sinal de controle malha fechada
%     u1 = vr(k)*cos(eb(3)) + u(1); 
%     u2 = wr(k) +  u(2);
       
    
%     p = robot_model(p,[u1 u2]+,Ts); %fecha malha de posicao

    
    P = [P;p];
    U = [U;[u1 u2]];
    P_noise = [P_noise; p_noise];
    P_measure = [P_measure; p_measure];
    P_est = [P_est p_est];
    P_odo = [P_odo;p_odo];
end
figure(1)
plot(P_measure(:,1),P_measure(:,2),'red*','Linewidth',1)
hold on
plot(P_odo(:,1),P_odo(:,2),'Linewidth',2)
plot(P_est(1,:),P_est(2,:),'green','linewidth',1)
grid on
% plot(P_noise(:,1),P_noise(:,2),'*','Linewidth',1)
legend('Medido','Odometria Pura','Estimado')


figure %bussola
plot(the_bussola,'red','Linewidth',1)
hold on
plot(P_est(3,:))
hold on
plot(P_odo(:,3))
grid on
legend('Medido','Estimado','Odometria')



function p = measurement_model(x,u,Ts)
    p = x;
end