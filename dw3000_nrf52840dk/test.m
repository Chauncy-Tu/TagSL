

close all;clear all;clc;

load position1.mat;

ancpos0=[0,0];
ancpos1=[1,1;-1,1;-1,-1;1,-1]*0.8;
p_t=[-0.8,0;-0.4,0.4];
p_m=[-0.7205,0.0338;mean(x),mean(y)];
figure;
p1=plot(ancpos0(1),ancpos0(2),'ro');
hold on;
p2=plot(ancpos1(:,1),ancpos1(:,2),'bo');
p3=plot(p_t(:,1),p_t(:,2),'g*');
p4=plot(p_m(:,1),p_m(:,2),'k*');

grid on;

legend([p1,p2,p3,p4],"Master Anchor","Slave Anchor","Tag p_t","Tag p_m");