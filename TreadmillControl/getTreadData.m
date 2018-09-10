function [distance, deltaTime] = getTreadData(memTread, treadSize)

if memTread.Data(1) == (treadSize-1)
    msg = getData(memTread, treadSize);
    % frame rate of optitrack data
    frameRate = msg(1);
    % current frame number
    frameID = msg(2);
    % treadmill belt speed
    treadSpeed = msg(3);
    % robot speed
    robotSpeed = msg(4);
    
    deltaTime = getDeltaTime(frameRate, frameID);
    velTotal = treadSpeed + robotSpeed;
    %calculate distance traveled
    distance = velTotal*deltaTime;
else
    distance = 0.0;
    deltaTime = 0.0;
end