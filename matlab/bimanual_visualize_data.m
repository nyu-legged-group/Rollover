clc;close all,clear all
fs=1000;
windowSize = 5;
load('0327BimanualStatic01_Left.mat')
load('0327BimanuaStatic01_Right.mat')

t =R(1,:);
R0=R(2,:)/5*1000;
L0=L(2,:)/5*1000;
offsetR = mean(R0(1:500));
offsetL = mean(L0(1:500));

%R = R0-offsetR;
%L = L0-offsetL;
R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);
L=filter(ones(1,windowSize)/windowSize,1,L0-offsetL);
%R=lowpass(R0-offsetR,0.125,fs);
%L=lowpass(L0-offsetL,0.125,fs);
%L=tarekLPF(L0-offsetL,100,fs);
%R=tarekLPF(R0-offsetR,100,fs);

figure(1)
plot(t,R)
xlim([6.750,7.225])
xlabel('t(s)')
ylabel('Force(N)')
legend('Right hand force')
title('Bimanual force')
figure(2)
plot(t,L)
xlim([6.750,7.225])
xlabel('t(s)')
ylabel('Force(N)')
legend('Left hand force')
title('Bimanual force')
if(max(R)>=max(L))
    bimanual_data=[t',R'];
    max(R)
else
    bimanual_data=[t',L'];
    max(L)
end
writematrix(bimanual_data,'0327BimanualStatic01.csv') 

%windowSize = 5;
%R=lowpass(R0-offsetR,0.125,fs);
%R=tarekLPF(R0-offsetR,0.125,fs);
%R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);
%L=tarekLPF(L0-offsetL,0.125,fs);
%L=lowpass(L0-offsetL,0.125,fs);
%L=filter(ones(1,windowSize)/windowSize,1,L0-offsetL);
