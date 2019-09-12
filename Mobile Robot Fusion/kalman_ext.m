% Argumentos 
%Ts -> tempo de amostragem
%f -> modelo estado
%h -> modelo saida
%Jf -> Jacobiana F
%Jh -> Jacobiana H
%Q -> Covarianca estado
%R -> Covarianca Saida
%x -> estado anterior
%u -> entrada 
%P -> covarianca erro
%y -> leitura
function [x_est,Kk,Pk] = kalman_ext(Ts,f,h,Q,R,x,u,Pk,y)

%Predict
x_est = f(x,u,Ts)';

Jf = [1 0 -sin(x(3))*Ts; %Jacobiana do processo f(x,u)
      0 1 cos(x(3))*Ts;
      0 0 1];



Pk = Jf*Pk*Jf'+Q;
%Update

Jh = eye(3); %Jacobianas da medição h(x)


Kk = Pk*Jh'/(Jh*Pk*Jh'+R);
x_est = x_est + Kk*(y - h(x_est,u,Ts));
Pk = (eye(length(Pk)) - Kk*Jh)*Pk;






end


