
t=ch0(1,:);
R=ch0(2,:);
L=ch1(2,:);

mean_R=mean(R);
mean_L=mean(L);
R1=R-mean_R; L1=L-mean_L;

ch={R',L'};
xlswrite('tempdata.xls', ch);

save data.mat L R
figure(1)
%plot(t,L/5*1000)
plot(t,R/5*1000)

%massR = mean_R-initial_massB                                                                                                                                                                                                                                                                                                                              
%massL = mean_L-initial_massA 