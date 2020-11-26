%varargin é o modelo de incertezas (usado / real)
function xk = robot_model(x0,u,Ts,varargin)
%xk -> [x,y,cos(theta)]

if(nargin == 3)
v_uncertain = u(1);
w_uncertain = u(2);     
else %usar no modelo "real"
v_uncertain = u(1) / varargin{1}.R;
w_uncertain = u(2) / varargin{1}.R/varargin{1}.D;        
end


xk(1) = x0(1) + v_uncertain*cos(x0(3))*Ts;
xk(2) = x0(2) + v_uncertain*sin(x0(3))*Ts;
xk(3) = x0(3) + w_uncertain*Ts;

xk = xk';


end