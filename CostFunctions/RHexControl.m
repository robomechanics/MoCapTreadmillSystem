function RHexControl(btObj, robotOn, angVel, gaitParams)
persistent lastOnCmd;
if isempty(lastOnCmd)
    lastOnCmd = false;
end

%% Check if robot control is on
if robotOn
    if lastOnCmd
        % -1 indicates no chnage in gair params and just updates
        % angVel on miniRHex
        cmdPacket = [-1 angVel gaitParams];
    else
        % 1 indicates chnage to gait params and updates gait
        % updating miniRHex gait restarts the gait also
        cmdPacket = [1 angVel gaitParams];
        %sets lastOnCmd to true
        lastOnCmd = robotOn;
    end
else
    %set data packet cmds to zero
    cmdPacket = [0 0 gaitParams];
    %sets lastOnCmd to false
    lastOnCmd = robotOn;
end

%% Update Robot
% construct data string to send to robot from cmdPacket
dataStr = ['<' int2str(cmdPacket(1)) ',' int2str(cmdPacket(2)) ',' ...
    int2str(cmdPacket(3)) ',' int2str(cmdPacket(4)) ',' int2str(cmdPacket(5)) ',' ...
    int2str(cmdPacket(6)) ',' int2str(cmdPacket(7)) ',' int2str(cmdPacket(8)) '>'];

%Send signal to robot
fprintf(btObj, dataStr);