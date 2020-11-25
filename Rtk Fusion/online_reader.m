clear;close all;clc
load('onlinefusion.mat')

plot(gps_data(:,2),gps_data(:,3))
hold on
plot(rtk_data(:,2),rtk_data(:,3),'*')
plot(fused_data(3:end,2),fused_data(3:end,3),'r')