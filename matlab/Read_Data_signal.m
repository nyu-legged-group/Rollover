save ch0.mat
save ch1.mat

A=ch0.signals.values;
B=ch1.signals.values;

SA=(size(A));SB=(size(B));
offsetA=mean(A(1,1:200));
offsetB=mean(B(1,1:200));
A1=A-offsetA; B1=B-offsetB;
A2=reshape(A1.',[1,SA(1)*SA(2)]);
B2=reshape(B1.',[1,SB(1)*SB(2)]);
t=[0:0.001:(SA(1)*SA(2)-1)*0.001];

ch={A2,B2};
xlswrite('tempdata.xls', ch);

save data.mat A2 B2
plot(t,A2,t,B2) 

