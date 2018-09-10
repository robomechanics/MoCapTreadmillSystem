%% Optirtrack/NatNet to Matlab Data Stream for Automated Treadmill Control
% written by Barrett Werner (bwerner@andrew.cmu.edu)
function OptitrackMatlabConnection()
global OptiFrameRate;
global moCapFile
global moCapSize
%% USER INPUTS
LocalIP = char('127.0.0.1'); % Enter your local IP address
ServerIP = char('127.0.0.1'); % Enter the server's IP address
% Must match robot and treadmill control gui
moCapFile = 'rigidBodyData.dat';
moCapSize = 10;

%% Setup NatNet
% Setup TCP Connection
theClient = SetupNatNetConnection(LocalIP,ServerIP);

% Get frame rate from Optitrack
[byteArray, retCode] = theClient.SendMessageAndWait('FrameRate');
if(retCode ==0)
    byteArray = uint8(byteArray);
    OptiFrameRate = typecast(byteArray,'single'); % (frame/sec)
    OptiFrameRate = double(OptiFrameRate);

end

% Add NatNet FrameReady event handler
ls = addlistener(theClient,'OnFrameReady2',@(src,event)FrameReadyCallback(src,event));
disp('[NatNet] FrameReady Listener added.');

% Wait for User Termination
run = true;
while(run)
    str = input('Press Enter To Exit Function ', 's');
    if (isempty(str))
        disp('Terminating Optritrack/Matlab Data Stream function.')
        run = false;
    end
end

% Shutdown NATNET Connection
theClient.Uninitialize();
delete(ls);
disp('[NatNet] Client Unintialized and Listener Deleted');
fileLoc = fullfile(tempdir, moCapFile);
fclose('all');
delete(fileLoc);
end

%% Process data in a NatNet FrameReady Event listener callback
function FrameReadyCallback(src, event)
    persistent lastFrameID;
    persistent memLoc;
    global OptiFrameRate;
    global moCapFile
    global moCapSize
    
    % Get lastest frame data
    frameOfData = event.data;
    frameID = double(frameOfData.iFrame); % int32
    
    if isempty(memLoc)
        % setup shared data file for mocap data
        memLoc = setupDataFile(moCapFile, moCapSize);
        lastFrameID = frameID;
    end

    % Put OptiFrameRate, frameID, position and orientation into array
    rbData = frameOfData.RigidBodies(1);

    rbArray = [OptiFrameRate, frameID, double(rbData.x), double(rbData.y), ...
        double(rbData.z), double(rbData.qx), double(rbData.qy), ...
        double(rbData.qz), double(rbData.qw)];
    
    
    frameDiff = (frameID - lastFrameID);
    % Check if new frame of data
    if frameDiff > 0
        % Send data to file
        sendData(rbArray,memLoc,moCapSize);
        lastFrameID = frameID;
    end
end
