load ch0.mat
load ch1.mat

t=ch0(1,:);
A=ch0(2,:);
B=ch1(2,:);

mean_A=mean(A);
mean_B=mean(B);
A1=A-mean_A; B1=B-mean_B;

ch={A,B};
xlswrite('tempdata.xls', ch);

save data.mat A B
figure(1)
plot(t,B)
title(Left)
print
initial_massA=mean_A
initial_massB=mean_B
figuer(2)
plot(t,B)
title(Right)                                                                                                                                                                                                                                                                                                                      