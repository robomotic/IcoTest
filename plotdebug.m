%data = load('ico.csv');
close all
figure
hold on
time=data(:,1);
plot(data(:,1),data(:,2),'r');
plot(data(:,1),data(:,3),'b');
legend('x0','x1');
save
figure
hold on
time=data(:,1);
plot(data(:,1),data(:,4),'r');
plot(data(:,1),[0;diff(data(:,4))],'g');
plot(data(:,1),data(:,5),'b');
legend('u0','delta u0','u1');
save
figure
hold on
plot(data(:,1),[0;diff(data(:,4))],'g');
plot(data(:,1),data(:,5),'b');
legend('delta u0','u1');
axis([min(time)  max(time) min(diff(data(:,4))) max(diff(data(:,4)))]);
figure
hold on
plot(data(:,1),data(:,6),'r');
plot(data(:,1),data(:,7),'b');
legend('left W','right W');