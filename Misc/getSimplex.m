function getSimplex(filename)
load(filename,'cost','gait');

[cost, ii] = sort(cost);
gait = gait(ii,:);
tempG = gait(4:5,1:2);
tempC = cost(4:5,:);
simplex = transpose(tempG);
simplex(end+1, :) = tempC;
save('simplex.mat', 'simplex');

