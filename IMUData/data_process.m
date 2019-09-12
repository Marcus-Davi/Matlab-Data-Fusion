clear;close all;clc
load('magdata')

 plot3(magdata(:,1),magdata(:,2),magdata(:,3),'*')
 hold on
 grid on
 
 MAX = max(magdata);
 MIN = min(magdata);
 
 offx = (MAX(1) + MIN(1)) / 2;
 offy = (MAX(2) + MIN(2)) / 2;
 offz = (MAX(3) + MIN(3)) / 2;
 
 magdata_hardiron_corrected = [magdata(:,1) - offx magdata(:,2) - offy magdata(:,3) - offz]
 
 plot3(magdata_hardiron_corrected(:,1),magdata_hardiron_corrected(:,2),magdata_hardiron_corrected(:,3),'*')
 