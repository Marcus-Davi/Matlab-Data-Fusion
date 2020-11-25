%varargin é o modelo de incertezas (usado / real)
function Jh = robot_jacobian2(x0,u,Ts,varargin)
%xk -> [x,y,cos(theta)]

if(nargin == 3)
v_uncertain = u(1);
w_uncertain = u(2);     
else %usar no modelo "real"
v_uncertain = u(1) / varargin{1}.R;
w_uncertain = u(2) / varargin{1}.R/varargin{1}.D;        
end

Jh = [1 Ts 0 0 0 0;
      0 0 0 0 -u(1)*sin(x0(5)) 0;
      0 0 1 Ts 0 0;
      0 0 0 0 u(1)*cos(x0(5)) 0;
      0 0 0 0 1 Ts;
      0 0 0 0 0 0];
      


end