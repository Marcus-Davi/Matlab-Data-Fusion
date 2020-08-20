function [xest,Pest] = ekalman_predict(x,u,P,Qn,model,jacobian,Ts)
%ekalman_predict(x,u,P,Qn,model,jacobian,Ts) - Predict non-linear model states
% x : states
% u : inputs
% P : error covariance 
% Qn : state covariance
% model : pointer to @model
% jacobian : pointer to @jacobian
% Ts : sampling time

xest = model(x,u,Ts);

Jf = jacobian(x,u,Ts);
Pest = Jf*P*Jf' + Qn;




end

