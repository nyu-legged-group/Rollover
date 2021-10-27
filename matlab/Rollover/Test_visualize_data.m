clc
%load('right_sensor.mat');
t =ch0(1,:);
R0=ch0(2,:)/5*1000;
L0=ch1(2,:)/5*1000;
offsetR = mean(R0(1:500));
offsetL = mean(L0(1:500));
windowSize = 5;
R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);
L=filter(ones(1,windowSize)/windowSize,1,L0-offsetL);

% R = R0-offsetR
% t = t(4700:5000)
% R = R(4700:5000)
plot(t,R,t,L)
xlabel('t(s)')
ylabel('Force(N)')
legend('Right hand force','Left hand force')
title('Testing force')


