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

%%
VARIABLES = 3;
XPOINTS = 3000;
position = 1;
time = 1;
x = (1:XPOINTS)';
xlabels = (1:XPOINTS);
y = zeros(XPOINTS,VARIABLES);
flushinput(s);
while(1)
    
    while(s.BytesAvailable==0)
    end
    
    read = fscanf(s,'%f %f %f\n\r')
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
     plot(x,y)
  

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





