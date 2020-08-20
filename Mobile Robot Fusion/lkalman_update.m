function [xest,Pest] = lkalman_update(y,x,u,P,Rn,SS,Ts)
%ekalman_predict(x,u,P,Qn,model,jacobian,Ts) - Predict non-linear model states
% x : states
% u : inputs
% P : error covariance 
% Qn : state covariance
% model : pointer to @model
% jacobian : pointer to @jacobian
% Ts : sampling time
err = y - SS.C*x;
S = SS.C*P*SS.C' + Rn;
K = P*SS.C'*inv(S);
xest = x + K*err;
Pest = (eye(length(P)) - K*SS.C)*P;





end

