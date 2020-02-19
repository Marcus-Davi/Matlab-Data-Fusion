function m = f(x,w,u)
q_k = x(1:4); %quart rotate
bias = x(5:end); %bias
w_measure = u(1:3);
bias_adj = u(4:end);
%x_k+1 = x_k + f(x_k)dt + G.udt
q_k1 = q_k + 1/2*quatmultiply(q_k,[0 w_measure]) -1/2 *quatmultiply(q_k,[0 bias]); 
bias_k1 = bias' - [0.1 0 0;0 0.1 0;0 0 0.1]*bias' + bias_adj';
m = [q_k1 bias_k1']';
end