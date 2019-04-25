
clear;
close all;

load 'trialData.mat'



figure(1)
plot(totalEnergy)
startTime = [1 592 1157];
endTime = [591 1156 1706];
for iT = 1:length(startTime)
    for ii = startTime(iT):endTime(iT)
        sumDist(iT,ii) = sum(dist(startTime(iT):ii));
        totalTime(iT,ii) = sum(pdt(startTime(iT):ii));
        cost(iT,ii) = (totalEnergy(ii))/sumDist(ii);

    end
end
figure(2)
hold on;
for iT = 1:length(startTime)
    plot(cost(iT,:))
end
