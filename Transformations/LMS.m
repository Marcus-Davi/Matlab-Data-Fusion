clear all;close all;clc

N = 50;
[x,y] = make_cloud(N,10,0,15,1);

% y = add_noise(y,0.1);

P0 = [x';y'];

T0 = [15;15];
% p0 = [x(50);y(50)];
p0 = [0;0];


figure
hold on
grid on
plot(P0(1,:),P0(2,:),'blue','linewidth',3);
for angle=-pi/6
R = [cos(angle) -sin(angle);sin(angle) cos(angle)];
P1 = p_transform(P0,R,T0,p0);
P1(2,:) = P1(2,:) + 0.05*randn(1,N);
plot(P1(1,:),P1(2,:),'black','linewidth',3);
pause(0.1)
end
% legend('P_0','P_1')
% return
%% Objetivo -> Estimar R e T que leva P0 a P1
%P1 = R*P0 + T
% SCAN MATCHING Já feito! (index mode)
% Função Custo -> F(R,T) = sum(R*p0 +T - p1)^2

%% Gauss Newton MT BOM!!!
%r(x) = R*Pi + T - Pi 
x = [0 0 0]';



tic
for k=1:25
    pause(0.10)
    R = [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
    T = [x(1);x(2)];
    r = [];
    jac = [];
for i=1:N
    r = [r;R*P0(:,i)+ T - P1(:,i)];
    jac = [jac ;Jf(P0(:,i),P1(:,i),x)];
end
     increment = -inv(jac'*jac)*jac'*r;
         x = x + 0.15*increment(:,1);
         
        
   R = [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
T = [x(1);x(2)];      
    Pnewton = p_transform(P0,R,T,p0);
plot(Pnewton(1,:),Pnewton(2,:),'--blue');

         
end




disp('Real')
disp(T0)
disp(angle)
disp('Estimado')
disp(x)
return
%% Gradiente Descendente
% gamma = 0.001;
% cost_old = 0;
% x = [0 0. 0]';
% err = 1;
% tol = 0.00001;
% iter = 0;
% while err>tol   
%     jac = 0;
%     cost = 0;
%     for i=1:N
%     cost = cost + F(P0(:,i),P1(:,i),[x(1) x(2) x(3)]');   
%     jac = jac + Jf(P0(:,i),P1(:,i),[x(1) x(2) x(3)]');
%     end
%     cost = cost/N;
%     jac = jac/N;
%     x = x - gamma*jac
%     err = abs((cost - cost_old))
%     cost_old = cost;
%     iter = iter+1;
% end
% iter
% Tsol = [x(1) x(2)]'
% Rsol = [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
% Psol = p_transform(P0,Rsol,Tsol,p0);
% plot(Psol(1,:),Psol(2,:));

%% Teste



%% Functions



function [x,y] = make_cloud(n,parts,x0,xf,rand_amp)
pts_part = round(n/parts);
y = [];
for i = 1:parts
y = [y;rand_amp*randn*ones(pts_part,1)];
end
x = linspace(x0,xf,n)';
y = y+rand_amp*10;
end



% Total trasnform
% P = [x x2 ... ; y1 y2 ...]
function P_t = p_transform(P,R,T,p)
P_t = R*(P-p) + T + p;
end  


% %usar vetor coluna
function y = F(p0,p1,x)
T = [x(1)  x(2)]';
R = [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
 y = norm(p0)^2 + norm(T)^2 + norm(p1)^2  + 2*p0'*R'*T - 2*p0'*R'*p1 - 2*T'*p1;
end

function y = Jf(p0,p1,x)
tx = x(1);
ty = x(2);
theta = x(3);
x0 = p0(1);y0=p0(2);
x1 = p1(1);y1=p1(2);

y = [1 0 -sin(theta)*x0-cos(theta)*y0;0 1 cos(theta)*x0-sin(theta)*y0];
end


function y = Jf2(p0,p1,x)
tx = x(1);
ty = x(2);
theta = x(3);
x0 = p0(1);y0=p0(2);
x1 = p1(1);y1=p1(2);

y = [2*tx + 2*cos(theta)*x0 - 2*sin(theta)*y0 + 2*x1;
     2*ty + 2*cos(theta)*y0 + 2*sin(theta)*x0 + 2*y1;
     2*(-sin(theta)*(x0*tx+y0*ty)+cos(theta)*(x0*ty-y0*tx) - ...
     sin(theta)*(x0*x1+y0*y1) + cos(theta)*(x0*y1-y0*x1))];

end