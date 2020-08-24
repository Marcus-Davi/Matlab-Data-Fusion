function [acc,ang] = ikinematic(p0,p1,Samples,Ts,alpha)
% Determine body frame accelerations when movin from pose 'p0' to pose 'p1'
% at sample time Ts

offset = 20;
%divide p0
x = linspace(p0(1),p1(1),Samples-2*offset)';
y = linspace(p0(2),p1(2),Samples-2*offset)';
z = linspace(p0(3),p1(3),Samples-2*offset)';

x = [p0(1)*ones(offset,1);x;p1(1)*ones(offset,1)];
y = [p0(2)*ones(offset,1);y;p1(2)*ones(offset,1)];
z = [p0(3)*ones(offset,1);z;p1(3)*ones(offset,1)];


xv = [x(2:end) - x(1:end-1);0]/Ts;
yv = [y(2:end) - y(1:end-1);0]/Ts;
zv = [z(2:end) - z(1:end-1);0]/Ts;

% filtrar velocidades
B = alpha;
A = [1 (alpha-1)];
xv_f = filter(B,A,xv);
yv_f = filter(B,A,yv);
zv_f = filter(B,A,zv);

xa = [xv_f(2:end) - xv_f(1:end-1); 0]/Ts;
ya = [yv_f(2:end) - yv_f(1:end-1); 0]/Ts;
za = [zv_f(2:end) - zv_f(1:end-1); 0]/Ts;



acc = [xa ya za]; %ENU
% acc = [ya xa -za]; %NED

ang = zeros(Samples,3);




end




