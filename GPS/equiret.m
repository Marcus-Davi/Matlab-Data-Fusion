function [x,y] = equiret(lat,lon,lat_r,lon_r)
C_EARTH = 6378137.0;
d_lon = lon - lon_r;
d_lat = lat - lat_r;
y = deg2rad(d_lat) * C_EARTH;
x = deg2rad(d_lon) * C_EARTH .* cos(deg2rad(lat));

end