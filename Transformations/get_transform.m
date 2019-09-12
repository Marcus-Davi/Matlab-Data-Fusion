%entra com as duas nuvem de pontos
function x = get_transform(P0,P1,x0)
N = length(P0);




%% Gauss Newton MT BOM!!!
%r(x) = R*Pi + T - Pi 
x = x0;

for k=1:5
%     pause(0.015)
    R = [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
    T = [x(1);x(2)];
    r = [];
    jac = [];
for i=1:N
    r = [r;R*P0(:,i)+ T - P1(:,i)]; %esta função deve ser alterada pra levar em conta a indexação
    jac = [jac ;Jf(P0(:,i),P1(:,i),x)];
end
     increment = -inv(jac'*jac)*jac'*r;
         x = x + increment(:,1);
                 
end


end


%% Functions



function y = Jf(p0,p1,x)
tx = x(1);
ty = x(2);
theta = x(3);
x0 = p0(1);y0=p0(2);
x1 = p1(1);y1=p1(2);

y = [1 0 -sin(theta)*x0-cos(theta)*y0;0 1 cos(theta)*x0-sin(theta)*y0];
end
