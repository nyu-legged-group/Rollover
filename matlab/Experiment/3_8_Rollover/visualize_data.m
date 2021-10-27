clc;close all

t =rRollover(1,:);
R0=rRollover(2,:)/5*1000;
offsetR = mean(R0(1:500));

windowSize = 5;
R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);

% R = R0-offsetR
% t = t(4700:5000)
% R = R(4700:5000)
plot(t,R)
xlabel('t(s)')
ylabel('Force(N)')
legend('Right hand force')
title('Rollover force')


