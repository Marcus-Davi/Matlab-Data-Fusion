%% Estabelecimento da Conexao Bluetooth
if exist('porta')
    fclose(porta);
end
clear ;close all;clc
disp('Conectando a Porta Serial...');
global porta
% porta = serial('/dev/ttyACM0','BaudRate',115200,'Terminator','CR');
porta = serial('/dev/rfcomm0','BaudRate',115200,'Terminator','CR');
fopen(porta);
disp('Conectado.');
Ts = 1/25;
Qn = 1e-5*eye(4);
Qn_v2 = 1e-5*eye(7);
Rn = 0.71e-3*eye(6); 
Rn(4,4) = 2;
Rn(5,5) = 2;
Rn(6,6) = 3;


Q_bias = Ts^2/2;

Qn_euler = [0.01 0 0 0 0 0;
    0 0.01 0 0 0 0;
    0 0 0.01 0 0 0;
    0 0 0 Ts^2/2 0 0;
    0 0 0 0 Ts^2/2 0;
    0 0 0 0 0 Ts^2/2];

Rn_euler = 0.20*eye(3);


XPOINTS = 600;
position = 1;
y_plot = zeros(XPOINTS,3);
% sim('simula')
% return
%% APENAS PLOTA O QUATERNION DA PLACA
flushinput(porta);
while(1)
    
    while(porta.BytesAvailable==0)
    end
    
    x = fscanf(porta,'%f %f %f %f\n\r')
if(length(x) ~= 4)
    x = zeros(1,4);
end 

    if (position < XPOINTS)
        position = position + 1;
    else
        position = 1;
    end
    
    v = [1 0 0];
    
%     eul = quat2eul(x');
%     y(position,1) = eul(1);
%     y(position,2) = eul(2);
%     y(position,3) = eul(3);
    
    if(mod(position,2)==0)
%     plot(y(1:position,:)) %plota euler
    
    
    
%     %plota vetor
    r = quatrotate(quatconj(x'),v);
%     subplot(2,1,1)
    plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
    plot3([0 0],[0 1],[0 0],'black','linewidth',2);
    plot3([0 0],[0 0],[0 1],'black','linewidth',2);
    plot3([0 r(1)],[0 r(2)],[0 r(3)],'blue','linewidth',2);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    hold off;

end
    
    drawnow;
    
end

% Pitch
