function y = euler_integral(data,Ts)

y = zeros(1,length(data))
for i=2:length(data)
y(i) = y(i-1)+data(i-1)*Ts;


end