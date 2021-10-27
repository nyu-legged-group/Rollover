clc;close all,clear all
fs=1000;
load('0327Rollover02_right_sensor.mat')

t =rRollover(1,:);
R0=rRollover(2,:)/5*1000;
offsetR = mean(R0(1:500));

R = R0-offsetR;

rollover_data=[t',R'];

%R=lowpass(R,0.125,fs);
R=tarekLPF(R,0.125,fs);

figure(1)
plot(t,R)
xlim([6.000,7.000])
xlabel('t(s)')
ylabel('Force(N)')
legend('Right hand force')
title('Rollover force')
writematrix(rollover_data,'Rollover_data02.csv') 

%windowSize = 5;
%R=lowpass(R0-offsetR,0.125,fs);
%R=tarekLPF(R0-offsetR,0.125,fs);
%R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);
%L=tarekLPF(L0-offsetL,0.125,fs);
%L=lowpass(L0-offsetL,0.125,fs);
%L=filter(ones(1,windowSize)/windowSize,1,L0-offsetL);
