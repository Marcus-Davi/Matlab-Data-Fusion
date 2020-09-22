function [x,y] = utm(lat,lon)
UTM_ZONE = 24;


falseEasting = 500e3;
falseNorthing = 10000e3;
k0 = 0.9996;
f = 1/298.257223563;
a = 6378137;
e = sqrt(f*(2-f));
n = f / (2-f);
n2 = n*n;n3=n2*n;n4=n3*n;n5=n4*n;n6=n5*n;
A = a/(1+n) * (1 + 1/4*n2 + 1/64*n4 + 1/256*n6);
Long0 = deg2rad(((UTM_ZONE-1)*6 - 180 + 3));

latitude = deg2rad(lat);
longitude = deg2rad(lon) - Long0;
cos_longitude = cos(longitude);
sin_longitude = sin(longitude);

alfa = [
		            1/2*n - 2/3*n2 + 5/16*n3 +   41/180*n4 -     127/288*n5 +      7891/37800*n6;
		                  13/48*n2 -  3/5*n3 + 557/1440*n4 +     281/630*n5 - 1983433/1935360*n6;
		                           61/240*n3 -  103/140*n4 + 15061/26880*n5 +   167603/181440*n6;
		                                   49561/161280*n4 -     179/168*n5 + 6601661/7257600*n6;
		                                                     34729/80640*n5 - 3418889/1995840*n6;
		                                                                  212378941/319334400*n6
                                                                          ];


t = tan(latitude);
sigma = sinh(e*atanh(e*t./sqrt(1+t.*t)));
t = t.*sqrt(1+sigma.*sigma) - sigma.*sqrt(1+t.*t);
% double t = sinh(atanh(sin(latitude))-2*sqrt(n)/(1+n)*atanh(2*sqrt(n)/(1+n)*sin(latitude)));
ksi_l = atan2(t,cos_longitude);
eta_l = atanh(sin_longitude./sqrt(t.*t + cos_longitude.*cos_longitude));

ksi = ksi_l;
eta = eta_l;

for j=1:6
	ksi = ksi + alfa(j)*sin(2*j*ksi_l).*cosh(2*j*eta_l);
	eta = eta + alfa(j)*cos(2*j*ksi_l).*sinh(2*j*eta_l);
end

x = falseEasting +  k0*A*eta; %// coordenadas UTM
y =  falseNorthing + k0*A*ksi;% //coordenadas UTM

% translada em relação à origem (gerar x,y coerentes)



end
