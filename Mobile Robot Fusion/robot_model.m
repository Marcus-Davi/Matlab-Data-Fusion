%p = robot position
function p = robot_model(p,u,Ts)
v= u(1);
w = u(2);

x = p(1);
y = p(2);
psi = p(3);

x = x + cos(psi)*v*Ts; 
y = y + sin(psi)*v*Ts; 
psi = psi + w*Ts;

p = [x y psi];
end