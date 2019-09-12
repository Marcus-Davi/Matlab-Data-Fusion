clear;close all;clc
nuvem0 = load('nuvem.txt');
nuvem0 = nuvem0(500000:550000,:);
pcshow(nuvem0);

x0 = nuvem0(:,1)';
y0 = nuvem0(:,2)';
z0 = nuvem0(:,3)';
onz = ones(1,length(nuvem0));
PC0 = [x0;y0;z0;onz];

yaw = 45;
pitch = 0;
roll = 0;

tx = 0;
ty = 0;
tz = 0;


Rroll = [1 0 0;0 cos(roll) sin(roll);0 -sin(roll) cos(roll)];
Rpitch = [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)];
Ryaw = [cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];

R = Ryaw*Rpitch*Rroll;
T = [1 0 0 tx;0 1 0 ty;0 0 1 tz;0 0 0 1];
T(1:3,1:3) = R

rotate_joint = [-15 122 10 0]';


PC1 = T*(rotate_joint-PC0) + rotate_joint;

%plot3([0 PC0(1)],[0 PC0(2)],[0 PC0(3)],'red','linewidth',2); 
%hold on;
%plot3([0 PC1(1)],[0 PC1(2)],[0 PC1(3)],'blue','linewidth',2); 
%grid on

nuvem1 = PC1';
nuvem1 = nuvem1(:,1:3);
hold on
pcshow(nuvem1)
