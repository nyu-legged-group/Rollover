function [y]=tarekLPF(y,o,cut)
%y:input signal
% o: order of the filter
% cut: cutoff frequency
x=fft(y);%fast fourier transform
po=floor(length(y)/2);% cutting fro left and wright
for i=(po-cut):(po+cut)
    x(i)=(x(i)/o);
end 

f=x;
y=ifft(f,'symmetric');
end