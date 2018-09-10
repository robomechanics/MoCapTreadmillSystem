load('optData.mat');
[cost, ii] = sort(cost);
gait = gait(ii,:);
tempG = gait(2:8,:);
tempC = cost(2:8,:);
simplex = transpose(tempG);
simplex(end+1, :) = tempC;
save('simplex.mat', 'simplex');

