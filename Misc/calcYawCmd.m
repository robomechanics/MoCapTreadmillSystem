function [dataOut, handles] = calcYawCmd(handles)

persistent cmdData;
persistent lastFrameID;

% Check if data file is ready to be read
if handles.memMoCap.Data(1) == (handles.moCapSize-1)
    
    %Read in data from Optitrack
    [frameRate, frameID, pos, eulerDeg] = getMoCapData(handles.memMoCap, handles.moCapSize); 
    
    % Init frame id
    if isempty(lastFrameID)
        lastFrameID = frameID;
    end
    
    dt = getDeltaTime(frameRate, frameID, lastFrameID);
    lastFrameID = frameID;
    % Set text displays to view data
    set(handles.textXpos, 'String', pos(1));
    set(handles.textYpos, 'String', pos(2));
    set(handles.textZpos, 'String', pos(3));
    set(handles.textXang, 'String', eulerDeg(1));
    set(handles.textYang, 'String', eulerDeg(2));
    set(handles.textZang, 'String', eulerDeg(3));


    % Yaw Angular Velocity Controller
    % Yaw and Z pos Error
    eYaw = handles.yawRef - eulerDeg(2);
    eZ = handles.zRef - pos(3);

    % Calcuate Yaw and Z pos error deriviatives
    [eDotZ, eDotYaw] = getErrDot(5,eZ,eYaw,dt);

    % Attitude and Position Commands
    attCmd = -handles.KpYaw * eYaw + handles.KdYaw * eDotYaw;
    posCmd = handles.KpZpos * eZ + handles.KdZpos * eDotZ;
    % Total Yaw Angular Velocity Command
    angVelCmd = attCmd + posCmd;

    % Check if in limits
    if abs(angVelCmd) > handles.angVelLimit
        if angVelCmd >= 0
            angVel = handles.angVelLimit;
        else
            angVel = -handles.angVelLimit;
        end
    else
        angVel = angVelCmd;
    end

    % Set text displays to view data, for gain tuning
    set(handles.text29, 'String', attCmd);
    set(handles.text30, 'String', posCmd);
    set(handles.text31, 'String', angVel);

    % Save data
    cmdData.cmd = angVel;
    cmdData.posError = eZ;
    cmdData.attError = eYaw;
    cmdData.posErrDot = eDotZ;
    cmdData.attErrDot = eDotYaw;
else
    % Init values
    if isempty(cmdData)
        cmdData.cmd = 0.0;
        cmdData.posError = NaN;
        cmdData.attError = NaN;
        cmdData.posErrDot = NaN;
        cmdData.attErrDot = NaN;
    end
end

dataOut = cmdData;

