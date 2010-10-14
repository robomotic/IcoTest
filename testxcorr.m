u0=data(:,4);
u1=data(:,5);
t=linspace(0,10,400);
smooth=exp(-t/8)';
u00=u0.*smooth;

close all
subplot(2,1,1)
plot(xcorr(u1,u0))
subplot(2,1,2)
hold on
plot(xcorr(u1,u00),'r')
%plot(xcorr(u1,u0),'k')
legend('learning','u0')
figure
subplot(2,1,1)
hold on
hold on
plot(u1,'k')
plot(u0,'r')
legend('u1','u0')
subplot(2,1,2)
hold on
plot(u1,'k')
plot(u00,'r')
legend('u1','u0')