function packet = treadmillCommand(vL,vR,aL,aR,incline)
% Gets send command for treadmill

aux = int16toBytes(round([vR*1000,vL*1000,0,0,aR*1000,aL*1000,0,0,0]));
actualData = reshape(aux',size(aux,1)*2,1);
secCheck = 255 - actualData; padding = zeros(1,27);
packet = [0 actualData' secCheck' padding];