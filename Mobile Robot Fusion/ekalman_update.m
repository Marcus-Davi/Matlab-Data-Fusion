function [xest,Pest] = ekalman_update(y,x,u,P,Rn,model,jacobian,Ts)
%ekalman_predict(x,u,P,Qn,model,jacobian,Ts) - Predict non-linear model states
% y : measurement
% x : states
% u : inputs
% P : error covariance 
% Qn : state covariance
% model : pointer to @model
% jacobian : pointer to @jacobian
% Ts : sampling time

yest = model(x,u,Ts);

Jh = jacobian(x,u,Ts);
err = y - yest;
S = Jh*P*Jh' + Rn;
K = P*Jh'*inv(S);
xest = x + K*err;
Pest = (eye(length(P)) - K*Jh)*P;




end

