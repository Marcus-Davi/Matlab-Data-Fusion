
clear;close all;clc

load('acc_data')

g_z = 800; %247
g_zn = -259.76;

g_x = 254.2;
g_xn = -245;

g_y = 251;
g_yn = -240;

ref = 250;


off_x = round((g_x + g_xn)/2)
off_y = round((g_y + g_yn)/2)
off_z = round((g_z + g_zn)/2)

scale_x = (2*ref/(g_x-g_xn))
scale_y = (2*ref/(g_y-g_yn))
scale_z = (2*ref/(g_z-g_zn))