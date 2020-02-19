%% Estabelecimento da Conexao Bluetooth
if exist('s')
    fclose(s);
end
clear ;close all;clc
disp('Conectando a porta COM...');
%b =
%serial('COM9','BaudRate',115200,'Terminator','CR','InputBufferSize',50000);%windows
% s = serial('/dev/ttyACM0','BaudRate',115200,'Terminator','CR','InputBufferSize',50000);
s = serial('/dev/rfcomm0');
fopen(s);
disp('Conectado.')

vec = [1 0 0]';

%%
VARIABLES = 5;
XPOINTS = 100;
position = 1;
time = 1;
x = (1:XPOINTS)';
xlabels = (1:XPOINTS);
y = zeros(XPOINTS,VARIABLES);
flushinput(s);
while(1)
    
    while(s.BytesAvailable==0)
    end
    
    read = fscanf(s,'%f %f %f %f %f\n\r')
% read = fscanf(s,'%d %d %d %d %d %d %d %d %d\n\r');
%     read = fscanf(s,'%f %f %f %f %f %f %f %f %f\n\r'); %

    xlabels(position) = time;
    time = time + 1;
    if (position < XPOINTS)
        position = position + 1;
    else
%         break
        position = 1;
    end
    
    for i=1:VARIABLES
       y(position,i) = read(i); 
    end
    
    
    drawnow;
%      plot(x,y(:,3:end))
     
phi = read(5);
theta = -read(4);
psi = read(3);
     
R_psi = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)];
R_theta = [cos(theta) 0  sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
R_phi = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1]; %rotação regra-mao-direita

%BODY 2 NAV! %Matlab precedencia comeca pela direita
R_B2N = R_psi*R_theta*R_phi; %Modelo ZYX 
R_N2B = R_psi'*R_theta'*R_phi'; %Modelo ZYX

vrot = R_N2B*vec;
    plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
    plot3([0 0],[0 1],[0 0],'black','linewidth',2);
    plot3([0 0],[0 0],[0 1],'black','linewidth',2);
    plot3([0 vrot(1)],[0 vrot(2)],[0 vrot(3)],'blue','linewidth',2);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    hold off;
    drawnow;

    grid on
    
end
%% salva dados MAG
magdata = [ y(1:position,1) y(1:position,2) y(1:position,3) ];
save('magdata')
% %% PITCH
% close all
% t = 1:position;
% a_pitch = y(t,1);
% g_pitch = y(t,2);
% f_pitch = y(t,3);
% 
% a_roll = y(t,4);
% g_roll = y(t,5);
% f_roll = y(t,6);
% 
% m_yaw = y(t,7);
% g_yaw = y(t,8);
% f_yaw = y(t,9);
% 
% plot(t,a_pitch)
% hold on
% plot(t,g_pitch)
% plot(f_pitch)
% 
% figure
% plot(t,a_roll)
% hold on
% plot(t,g_roll)
% plot(f_roll)
% 
% figure
% plot(t,m_yaw)
% hold on
% plot(t,g_yaw)
% plot(f_yaw)
% 
% fusiondata.a = [a_pitch a_roll];
% fusiondata.g = [g_pitch g_roll g_yaw];
% fusiondata.m = m_yaw;
% fusiondata.f = [f_pitch f_roll f_yaw];
% 
% save('fusiondata','fusiondata')





