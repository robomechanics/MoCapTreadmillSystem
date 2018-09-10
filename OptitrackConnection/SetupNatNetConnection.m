function [theClient] = SetupNatNetConnection(LocalIP,ServerIP)
% This function sets up a connection between Optitrack(1.10.0) and Matlab
% using NatNet SDK(2.10)

% Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
% Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
% are both in the same folder and/or path if you move them.
disp('[NatNet] Creating Client.')
% Find NatNet SDK DLL
% Place the NatNet SDK folder in same directory as this function
curDir = fileparts(mfilename('fullpath'));
dllPath = fullfile(curDir,'NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);

% Create an instance of a NatNet client
theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
version = theClient.NatNetVersion();
fprintf( '[NatNet] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4));

% Connect to an OptiTrack server (e.g. Motive)
disp('[NatNet] Connecting to OptiTrack Server.')
if (ServerIP == '0')
    % Connect to a local stream.
    flg = theClient.Initialize(LocalIP, LocalIP); % flg = returnCode: 0 = Success
else
    % Connect to another computer's stream.
    flg = theClient.Initialize(LocalIP, ServerIP); % flg = returnCode: 0 = Success
end

if (flg == 0)
    disp('[NatNet] Initialization Succeeded')
else
    disp('[NatNet] Initialization Failed')
end





 