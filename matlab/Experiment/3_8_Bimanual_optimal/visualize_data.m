clc;close all

t =R(1,:);
R0=R(2,:)/5*1000;
L0=R(2,:)/5*1000;
offsetR = mean(R0(1:500));
offsetL = mean(R0(1:500));

windowSize = 5;
R=filter(ones(1,windowSize)/windowSize,1,R0-offsetR);
L=filter(ones(1,windowSize)/windowSize,1,L0-offsetR);

% R = R0-offsetR
% t = t(4700:5000)
% R = R(4700:5000)
figure(1)
plot(t,R)
xlabel('t(s)')
ylabel('Force(N)')
legend('Right hand force')
title('Bimanual force')
figure(2)
plot(t,L)
xlabel('t(s)')
ylabel('Force(N)')
legend('Left hand force')
title('Bimanual force')


