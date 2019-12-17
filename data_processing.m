clc,clear

%data = readmatrix('regre_500_1.csv');
data = readmatrix('regre.csv');
data = data(2:end,:)

x = data(:,2)
y = data(:,3)
z = data(:,4)

pre_x = data(:,5)
goodIdx = find(pre_x < 0.25)

pre_x = pre_x(goodIdx)
x = x(goodIdx)
z = z(goodIdx)

badIdx = find(x < 0.88 & pre_x > 0.030)
pre_x(badIdx) = []
x(badIdx) = []
z(badIdx) = []


badIdx = find(x < 0.8 & pre_x > -0.1)
pre_x(badIdx) = []
x(badIdx) = []
z(badIdx) = []




figure(8);clf;
hold on;
grid on;
scatter(x,pre_x)
ylim([-0.5,0.4])
xlim([0.7,1.0])

fit = polyfit(x,pre_x,1)
plot(x,fit(1)*x+fit(2),'lineWidth',5)
xlabel('initial velocity (m/s)')
ylabel('second bounce location (m)')
hold off;

figure(9);clf;
hold on;
grid on;
scatter(z,pre_x)
ylim([-0.5,0.5])
xlim([6.0,7.3])

fit = polyfit(z,pre_x,1)
plot(z,fit(1)*z+fit(2),'lineWidth',5)
xlabel('initial velocity (m/s)')
ylabel('second bounce location (m)')
hold off;



tbl = table(x,z,pre_x,'VariableNames',...
{'x','z','pre_x'});
mdl = fitlm(tbl)
figure(1)
plotResiduals(mdl,'fitted')




