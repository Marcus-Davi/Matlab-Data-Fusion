clear; close all;clc
zone = 24;
format long
f = 1/298.257223563
a = 6378137.0
n = f/(2-f)
n2 = n^2
n3 = n^3
n4 = n^4
n5 = n^5
n6 = n^6
A = a/(1+n) * (1 + 1/4*n2 + 1/64*n4 + 1/256*n6)
long_0 = ((zone-1.0)*6.0 - 180.0 + 3.0)/180*pi
alfa_1 = 1/2*n - 2/3*n2 + 5/16*n3
alfa_2 = 13/48*n2 -  3/5*n3
alfa_3 = 61/240*n3