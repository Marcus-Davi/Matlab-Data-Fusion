function [xest,Pest] = lkalman_predict(x,u,P,Qn,SS)
%ekalman_predict(x,u,P,Qn,model,jacobian,Ts) - Predict non-linear model states
% x : states
% u : inputs
% P : error covariance 
% Qn : state covariance
% model : pointer to @model
% jacobian : pointer to @jacobian
% Ts : sampling time

xest = SS.A*x + SS.B*u;
Pest = SS.A*P*SS.A' + Qn;




end

