clear;close all;clc

IMU = imuSensor('accel-gyro');
%% Acc parameters
g = 9.8066;
IMU.Accelerometer.MeasurementRange = 4*g;
IMU.Accelerometer.Resolution = (0.488)*(g/1000);
IMU.Accelerometer.NoiseDensity = 526*(g/1e6);
IMU.Accelerometer.TemperatureBias = 0.2*(g/1000);
IMU.Accelerometer.RandomWalk = [0 0 0];
IMU.Accelerometer.BiasInstability = [0.1 0.1 0];

%% Gyr Parameters
IMU.Gyroscope.MeasurementRange = deg2rad(500);
IMU.Gyroscope.Resolution = deg2rad(15.625*(1/1000));
IMU.Gyroscope.NoiseDensity = deg2rad(0.025);
IMU.Gyroscope.TemperatureBias = deg2rad(0.02);


%% IMU Parameters & Kinematic Model
disp(IMU.Accelerometer)
disp(IMU.Gyroscope)
Ts = 1/50;
IMU.SampleRate = 1/Ts;
IMU.Temperature = 25;

traj = kinematicTrajectory('SampleRate',1/Ts);


%% Simulation
Time = 10; %s
Samples = Time/Ts;
t = linspace(0,Time,Samples);
acc = zeros(Samples,3); %world acc
gyr = zeros(Samples,3); % world gyr


% Trajectory (World frame)
p0 = [0 0 0]';
p1 = [1 1 0]';
[acc_real,ang_real] = ikinematic(p0,p1,Samples,Ts,0.1);

% validade
traj = kinematicTrajectory('SampleRate',1/Ts);
[p,q] = traj(acc_real,ang_real);
% p
% return


eul = zeros(Samples,3); %rpy
q = quaternion(eul,'euler','XYZ','frame');


[acc_r,gyr_r] = IMU(acc_real,ang_real,q); %recebe NED

% plot(t,acc_r)
% hold on
% plot(t,acc_real)
% grid on
% legend('imu_x','real-x');

% return
%% Model
x = [p0(1) 0 p0(2) 0]';
X = [];
for i=1:Samples
    u = [-acc_r(i,1) -acc_r(i,2)]'
    x = model(x,u,Ts);
    X = [X;x'];
    
end

figure
plot(p(:,1),p(:,2))
hold on
plot(X(:,1),X(:,2))
legend('Real','Model')