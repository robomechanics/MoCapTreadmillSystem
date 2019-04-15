function varargout = MinitaurOptimizationControl(varargin)
% MINITAUROPTIMIZATIONCONTROL MATLAB code for MinitaurOptimizationControl.fig
%      MINITAUROPTIMIZATIONCONTROL, by itself, creates a new MINITAUROPTIMIZATIONCONTROL or raises the existing
%      singleton*.
%
%      H = MINITAUROPTIMIZATIONCONTROL returns the handle to a new MINITAUROPTIMIZATIONCONTROL or the handle to
%      the existing singleton*.
%
%      MINITAUROPTIMIZATIONCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MINITAUROPTIMIZATIONCONTROL.M with the given input arguments.
%
%      MINITAUROPTIMIZATIONCONTROL('Property','Value',...) creates a new MINITAUROPTIMIZATIONCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MinitaurOptimizationControl_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MinitaurOptimizationControl_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MinitaurOptimizationControl

% Last Modified by GUIDE v2.5 28-Feb-2019 15:08:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MinitaurOptimizationControl_OpeningFcn, ...
                   'gui_OutputFcn',  @MinitaurOptimizationControl_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end


% --- Executes just before MinitaurOptimizationControl is made visible.
function MinitaurOptimizationControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MinitaurOptimizationControl (see VARARGIN)

% USER INPUTS
% set to false to run gui without Optitrack Connection, for debugging only
handles.optiDataConnection = true;
%elliePi IP Address and Port
serverAddress = '128.237.228.243';
port = 50000;

% Setup TCP connection to robot
handles.tcpObj = tcpip(serverAddress,port); % TCP/IP object
if handles.tcpObj.ByteOrder ~= 'b'
    % use big endian for transmitting to python
    handles.tcpObj.ByteOrder = 'b';
end
fopen(handles.tcpObj); % Open the TCP/IP object
disp('Minitaur TCP open')

%Init States
handles.robotOn = false;
handles.mode = 1;
handles.pauseOpt = false;
handles.restartOpt = false;

%Init Values
handles.fwdVel = 0.0;
handles.angVel = 0.0;
handles.stHgt = 0.0;
%Optimization Variables
handles.P1 = 2.0; % stance height (extDes)
handles.P2 = 0.3; % min extenion in retraction (extMin)
handles.P3 = 0.009;
handles.P4 = 0.0;
handles.P5 = 0.0;
handles.P6 = 0.0;
handles.P7 = 0.0;
handles.numOptVars = 2;
handles.maxNumOptVars = 7;
handles.PUpBound = [2.5 .5 99999 99999 99999 99999 99999];
handles.PLowBound = [1 0 -99999 -99999 -99999 -99999 -99999];
%Treadmill Refernces
handles.zRef = 0.48;
handles.yawRef = 0.0;
handles.yawCenterLimit = 1.5; %deg
handles.zCenterLimit = 0.02; %meters
%Optimization Trial Variables
handles.timeToSS = 2.0; %sec
handles.trialLength = 5.0; %meters
handles.reverseDirection = true; %reverse positive direction of treadmill (for running backwards)

% Set initial gain values
handles.KpZpos = 0.09;
handles.KdZpos = 0.0;
handles.KpYaw = .001;
handles.KdYaw = 0.0;

% Set shared filenames and sizes
handles.moCapFile = 'rigidBodyData.dat';
handles.moCapSize = 10;
handles.treadFile = 'treadmillData.dat';
handles.treadSize = 5;

% Set optimization data filename
handles.optDataFile = 'optData';
handles.trialDataFile = 'trialData';

% Read in old simplex - used for Cont. Gait Opt only
% TODO: add load in after selecting Cont Gait Opt
%load('simplex.mat', 'simplex');
%handles.simplex = simplex;
handles.simplex = [];

% Update gui displays
set(handles.editZRef, 'String', handles.zRef);
set(handles.editYawRef, 'String',  handles.yawRef);
set(handles.editKpZpos, 'String', handles.KpZpos);
set(handles.editKdZpos, 'String',  handles.KdZpos);
set(handles.editKpYaw, 'String',  handles.KpYaw);
set(handles.editKdYaw, 'String',  handles.KdYaw);
set(handles.editP1, 'String',  handles.P1);
set(handles.editP2, 'String',  handles.P2);
set(handles.editP3, 'String',  handles.P3);
set(handles.editP4, 'String',  handles.P4);
set(handles.editP5, 'String',  handles.P5);
set(handles.editP6, 'String',  handles.P6);
set(handles.editP6, 'String',  handles.P7);

% Set Manual Control Limits
handles.fwdVelLimit = 1.0;
handles.angVelLimit = 0.03;

% Setup shared data file for mocap and treadmill data
if handles.optiDataConnection
    handles.memMoCap = setupDataFile(handles.moCapFile, handles.moCapSize);
    handles.memTread = setupDataFile(handles.treadFile, handles.treadSize);
end

%Create timer
handles.t_update = timer('TimerFcn',{@updateData,hObject}, 'Period',1/30,'ExecutionMode','fixedRate','BusyMode','queue');

% Choose default command line output for MinitaurOptimizationControl
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Start timer
start(handles.t_update);

% UIWAIT makes MinitaurOptimizationControl wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

function updateData(~, ~, hObject)
%get handles struct
handles = guidata(hObject);


if handles.mode == 3 || handles.mode == 4
    %In Optimization mode
    gaitOptimization(hObject);
else
    %In non-Opt mode
    if handles.optiDataConnection
        % Calculate yaw command and check if centered on treadmill
        [cmdData, handles] = calcYawCmd(handles);
    else
        cmdData.cmd = 0.0;
    end

    %Check Control Mode
    if handles.mode == 1
        %Manual Control Mode
        fwdVel = handles.fwdVel;
        angVel = handles.angVel;
    elseif handles.mode == 2
        %Optitrack Control Mode
        fwdVel = handles.fwdVel;
        angVel = cmdData.cmd;
    end

    optParams = [handles.P1 handles.P2 handles.P3 handles.P4 handles.P5 ...
        handles.P6 handles.P7];
    
    % Check if robot control is on
    if handles.robotOn
        cmdPacket = [fwdVel angVel optParams];
    else
        %set data packet cmds to zero
        cmdPacket = [0.0 0.0 optParams];
    end
    
    %Send signal to robot
    fwrite(handles.tcpObj, cmdPacket,'double');

end

% Update handles structure
guidata(hObject, handles);
end

function gaitOptimization(hObject)
%get handles struct
handles = guidata(hObject);

%stop timer to prevent interrupt during optimization
stop(handles.t_update);

% get initial values for optimization
x0 = [handles.P1 handles.P2];

% create anonymous function for x
costFunc = @(x)costFunction_Minitaur_speed(x,hObject);

% run optimization
if handles.mode == 3
    [finalGait, finalCost] = fminsearch(costFunc,x0);
elseif handles.mode == 4
    [finalGait, finalCost] = fminsearch_simplex(costFunc,handles.simplex);
else
    warning('ERROR: handles.mode at incorrect values for gait opt');
    finalGait = [0 0];
    finalCost = 0;
end

% display results
disp('Optimized Gait Parameters');
disp(finalGait);
disp('Cost of Optimized Gait Parameters');
disp(finalCost);

%restart timer
start(handles.t_update);

% Update handles structure
guidata(hObject, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = MinitaurOptimizationControl_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --- Executes on button press in BTstart.
function BTstart_Callback(hObject, eventdata, handles)
% hObject    handle to BTstart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.robotOn = true; 
set(handles.textOn_Off, 'String', 'Robot On');
% Update handles structure
guidata(hObject, handles);
end

% --- Executes on button press in BTstop.
function BTstop_Callback(hObject, eventdata, handles)
% hObject    handle to BTstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.robotOn = false; 
set(handles.textOn_Off, 'String', 'Robot Off');
% Update handles structure
guidata(hObject, handles);
end

function editFwdVel_Callback(hObject, eventdata, handles)
% hObject    handle to editFwdVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editFwdVel as text
%        str2double(get(hObject,'String')) returns contents of editFwdVel as a double

limit = handles.fwdVelLimit;
% Read user input
tempVal = str2double(get(hObject,'String')); 
%check if inputs is between bounds value
if tempVal >= limit
    tempVal = limit;
elseif tempVal <= -limit
    tempVal = -limit;
end
% save value
handles.fwdVel = tempVal;
% modifiy string to new value (might be limit value)
set(handles.editFwdVel, 'String', num2str(handles.fwdVel));
guidata(hObject, handles);
return
end

% --- Executes during object creation, after setting all properties.
function editFwdVel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editFwdVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editAngVel_Callback(hObject, eventdata, handles)
% hObject    handle to editAngVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAngVel as text
%        str2double(get(hObject,'String')) returns contents of editAngVel as a double

limit = handles.angVelLimit;
% Read user input
tempVal = str2double(get(hObject,'String')); 
%check if inputs is between bounds value
if tempVal >= limit
    tempVal = limit;
elseif tempVal <= -limit
    tempVal = -limit;
end
% save value
handles.angVel = tempVal;
% modifiy string to new value (might be limit value)
set(handles.editAngVel, 'String', num2str(handles.angVel));
guidata(hObject, handles);
return
end


% --- Executes during object creation, after setting all properties.
function editAngVel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAngVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editSH_Callback(hObject, eventdata, handles)
% hObject    handle to editSH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSH as text
%        str2double(get(hObject,'String')) returns contents of editSH as a double

%TODO: Find and set limits for this input
handles.stHgt = str2double(get(hObject,'String')); 
guidata(hObject, handles);
return
end

% --- Executes during object creation, after setting all properties.
function editSH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on selection change in popupCtrlMode.
function popupCtrlMode_Callback(hObject, eventdata, handles)
% hObject    handle to popupCtrlMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupCtrlMode contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupCtrlMode
str = get(hObject, 'String');
val = get(hObject, 'Value');
switch str{val}
case 'Manual Control' 
    handles.mode = 1;
case 'Optitrack Angular Control'
    handles.mode = 2;
case 'Gait Optimization'
    handles.mode = 3;
    handles.optData = matfile(handles.optDataFile, 'Writable', true);
    handles.optData.cost = 0;
    handles.optData.gait = [0 0 0 0 0 0 0];
    handles.trialData = matfile(handles.trialDataFile, 'Writable', true);
    handles.trialData.dt = 0;
    handles.trialData.dist = 0;
    handles.trialData.energy = 0;
    handles.trialData.voltage = 0;
    handles.trialData.current = 0;
    handles.trialData.totalEnergy = 0;
    handles.trialData.pdt = 0;
case 'Cont. Gait Optimization'
    handles.mode = 4;
    handles.optData = matfile(handles.optDataFile, 'Writable', true);
    handles.optData.cost = 0;
    handles.optData.gait = [0 0 0 0 0 0 0];
    handles.trialData = matfile(handles.trialDataFile, 'Writable', true);
    handles.trialData.dt = 0;
    handles.trialData.dist = 0;
    handles.trialData.energy = 0;
    handles.trialData.voltage = 0;
    handles.trialData.current = 0;
    handles.trialData.totalEnergy = 0;
    handles.trialData.pdt = 0;
end
guidata(hObject, handles);
end         

% --- Executes during object creation, after setting all properties.
function popupCtrlMode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupCtrlMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in BTcloseTCP.
function BTcloseTCP_Callback(hObject, eventdata, handles)
% hObject    handle to BTcloseTCP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fclose(handles.tcpObj);
disp('Minitaur TCP close')
% TODO: move this to close functions?
% Stop and delete timer to prevent warnin on next run
stop(handles.t_update);
delete(handles.t_update);

% Update handles structure
guidata(hObject, handles);
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%stop(handles.t_update);
%delete(handles.t_update);
%fclose(handles.tcpObj);

% Update handles structure
guidata(hObject, handles);

% Hint: delete(hObject) closes the figure
delete(hObject);
end

function editP1_Callback(hObject, eventdata, handles)
% hObject    handle to editP1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP1 as text
%        str2double(get(hObject,'String')) returns contents of editP1 as a double
handles.P1 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editP2_Callback(hObject, eventdata, handles)
% hObject    handle to editP2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP2 as text
%        str2double(get(hObject,'String')) returns contents of editP2 as a double
handles.P2 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editP4_Callback(hObject, eventdata, handles)
% hObject    handle to editP4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP4 as text
%        str2double(get(hObject,'String')) returns contents of editP4 as a double
handles.P4 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editP5_Callback(hObject, eventdata, handles)
% hObject    handle to editP5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP5 as text
%        str2double(get(hObject,'String')) returns contents of editP5 as a double
handles.P5 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editP6_Callback(hObject, eventdata, handles)
% hObject    handle to editP6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP6 as text
%        str2double(get(hObject,'String')) returns contents of editP6 as a double
handles.P6 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editP3_Callback(hObject, eventdata, handles)
% hObject    handle to editP3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP3 as text
%        str2double(get(hObject,'String')) returns contents of editP3 as a double
handles.P3 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editZRef_Callback(hObject, eventdata, handles)
% hObject    handle to editZRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editZRef as text
%        str2double(get(hObject,'String')) returns contents of editZRef as a double
handles.zRef = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editZRef_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editZRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function editKpYaw_Callback(hObject, eventdata, handles)
% hObject    handle to editKpYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKpYaw as text
%        str2double(get(hObject,'String')) returns contents of editKpYaw as a double
handles.KpYaw = str2double(get(hObject,'String')); 
guidata(hObject, handles)
end

% --- Executes during object creation, after setting all properties.
function editKpYaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKpYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function editKpZpos_Callback(hObject, eventdata, handles)
% hObject    handle to editKpZpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKpZpos as text
%        str2double(get(hObject,'String')) returns contents of editKpZpos as a double
handles.KpZpos = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKpZpos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKpZpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editYawRef_Callback(hObject, eventdata, handles)
% hObject    handle to editYawRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editYawRef as text
%        str2double(get(hObject,'String')) returns contents of editYawRef as a double
handles.yawRef = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editYawRef_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editYawRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editKdZpos_Callback(hObject, eventdata, handles)
% hObject    handle to editKdZpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKdZpos as text
%        str2double(get(hObject,'String')) returns contents of editKdZpos as a double
handles.KdZpos = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKdZpos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKdZpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editKdYaw_Callback(hObject, eventdata, handles)
% hObject    handle to editKdYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKdYaw as text
%        str2double(get(hObject,'String')) returns contents of editKdYaw as a double
handles.KdYaw = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKdYaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKdYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in BTpause.
function BTpause_Callback(hObject, eventdata, handles)
% hObject    handle to BTpause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~handles.pauseOpt
    handles.pauseOpt = true; 
    set(handles.BTpause, 'String', 'Resume');
else
    handles.pauseOpt = false; 
    set(handles.BTpause, 'String', 'Pause');
end
% Update handles structure
guidata(hObject, handles);
end

% --- Executes on button press in BTrestart.
function BTrestart_Callback(hObject, eventdata, handles)
% hObject    handle to BTrestart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.restartOpt = true; 
% Update handles structure
guidata(hObject, handles);
end



function editP7_Callback(hObject, eventdata, handles)
% hObject    handle to editP7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP7 as text
%        str2double(get(hObject,'String')) returns contents of editP7 as a double
handles.P7 = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editP7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
