clc;close all;

if exist('s') %#ok<EXIST> %Testa pra não dar LOCK do arquivo
    fclose(s);
end
clear;

s = serial('/dev/ttyACM0','BaudRate',115200,'Terminator','CR');
fopen(s);

%% Variavel Gráfico
XPOINTS = 100;
VARIABLES = 3;
x = (1:XPOINTS)';
y = zeros(XPOINTS,VARIABLES);

y_angles = y;
position = 1;

%% Sensors Gains
Ts = 1/50;
g = 9.80655;

ACC_GAIN = 0.488e-3;
GYR_GAIN = 15.625e-3 * pi/180.0;
MAG_GAIN = 0.1;

%% Calibrations (AN4399)

MAX_X = 2082 ; MIN_X = -2003;
MAX_Y = 2027 ; MIN_Y = -2108;
MAX_Z = 2102; MIN_Z = -1920;

Wxx = (2/0.488e-3 )/(MAX_X - MIN_X);
Wyy = (2/0.488e-3 )/(MAX_Y - MIN_Y);
Wzz = (2/0.488e-3 )/(MAX_Z - MIN_Z);

Vx = -g*(MAX_X + MIN_X)/(MAX_X - MIN_X);
Vy = -g*(MAX_Y + MIN_Y)/(MAX_Y - MIN_Y);
Vz = -g*(MAX_Z + MIN_Z)/(MAX_Z - MIN_Z);

W = diag([Wxx Wyy Wzz])
V = [Vx;Vy;Vz]

% MAX_Z * ACC_GAIN * g * Wzz + Vz
% MIN_Z * ACC_GAIN * g * Wzz + Vz

% return

%% Leitura
figure

grid on
flushinput(s)
while(1)
    leitura = fscanf(s,'%f %f %f');
    
       ACC = leitura(1:3)
       ACC_CAL = (W * ACC * ACC_GAIN * g) + V
      
       

%     GYR = leitura(4:6) * GYR_GAIN;
%     MAG = leitura(7:9) * MAG_GAIN;
    
    PLOT = ACC;
%     PLOT = GYR * GYR_GAIN;
%     PLOT = MAG * MAG_GAIN;
    
    y(position,1) = PLOT(1);
    y(position,2) = PLOT(2);
    y(position,3) = PLOT(3);
    
    
    %ANGLES COMPUTATION
%     a_p = getAccPitch(ACC);
%     a_r = getAccRoll(ACC);
    
    
    
    
    position = position + 1;
    if(position == XPOINTS)
        position = 1;
    end
    
    plot(x,y) 
%     plot(x,y_angles) 
    grid on
    drawnow
    
    
end



