%varargin é o modelo de incertezas (usado / real)
function xk = robot_model2(x0,u,Ts,varargin)
%xk -> [x,y,cos(theta)]

if(nargin == 3)
v_uncertain = u(1);
w_uncertain = u(2);     
else %usar no modelo "real"
v_uncertain = u(1) / varargin{1}.R;
w_uncertain = u(2) / varargin{1}.R/varargin{1}.D;        
end

%vels
xk(2) = v_uncertain*cos(x0(5));
xk(4) = v_uncertain*sin(x0(5));
xk(6) = w_uncertain;

%pos
xk(1) = x0(1) + xk(2)*Ts;
xk(3) = x0(3) + xk(4)*Ts;
xk(5) = x0(5) + xk(6)*Ts;

xk = reshape(xk,[length(x0),1]);


end