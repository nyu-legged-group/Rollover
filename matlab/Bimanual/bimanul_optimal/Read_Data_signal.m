%load ch0.mat
%load ch1.mat

fs=1e3;
t=ch0(1,:);
R=ch0(2,:);
L=ch1(2,:);
%R1=lowpass(R,150,fs);
%L1=lowpass(L,150,fs);

mean_R=mean(R);
mean_L=mean(L);
R1=R-mean_R; L1=L-mean_L;

ch={R',L'};
xlswrite('tempdata.xls', ch);

save data.mat L R
figure(1)
plot(t,L)
figure(2)
plot(t,R)

%massR = mean_R-initial_massB                                                                                                                                                                                                                                                                                                                              
%massL = mean_L-initial_massA 