
clear;
close all;

load 'trialDataTest1.mat'

figure(1)
plot(totalEnergy)

for ii = 1:292
    sumDist(ii) = sum(dist(1:ii));
    totalTime(ii) = sum(pdt(1:ii));
    cost(ii) = (totalEnergy(ii)/totalTime(ii))/sumDist(ii);
    
end
figure(2)
plot(cost)
