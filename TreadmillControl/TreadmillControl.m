function varargout = TreadmillControl(varargin)
% TREADMILLCONTROL MATLAB code for TreadmillControl.fig
%      TREADMILLCONTROL, by itself, creates a new TREADMILLCONTROL or raises the existing
%      singleton*.
%
%      Hfind = TREADMILLCONTROL returns the handle to a new TREADMILLCONTROL or the handle to
%      the existing singleton*.
%
%      TREADMILLCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TREADMILLCONTROL.M with the given input arguments.
%
%      TREADMILLCONTROL('Property','Value',...) creates a new TREADMILLCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TreadmillControl_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TreadmillControl_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TreadmillControl

% Last Modified by GUIDE v2.5 16-Aug-2018 00:56:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TreadmillControl_OpeningFcn, ...
                   'gui_OutputFcn',  @TreadmillControl_OutputFcn, ...
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


% --- Executes just before TreadmillControl is made visible.
function TreadmillControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TreadmillControl (see VARARGIN)

handles.hTreadmill = tcpip('localhost',4000); % TCP/IP object
set(handles.hTreadmill,'InputBufferSize',32,'OutputBufferSize',64); % Settings required to communicate with treadmill
fopen(handles.hTreadmill); % "Open the TCP/IP object
% Choose default command line output for KDTreadmill
disp('treadmill TCP open')
handles.state = 2;
handles.mode = 3;
handles.velocity = 0;
handles.peakVel = 0;
handles.minVel = 0;
handles.period = 0;
handles.optiCmd = 0;
handles.rec = false;
t = 0;

% Init Optitrack Controller Gains
handles.Kp = 2.0;
handles.Kd = 0.07;
handles.Ki = 0.11;
%miniRHex Controller Gains
%Kp = 2.0, Kd = 0.05, Ki = 0.1 %old gains, controller has changed since

% Init Optitrack Reference Positions
handles.xRef = -0.3; %meters
handles.yRef = 0.0; %meters
handles.zRef = 0.0; %meters

% Init Treadmill Size Info (shutoffs treadmill if robot outside this area)
handles.centerX = 0.0; %meters
handles.centerZ = 0.48; %meters
handles.treadLength = 2.0320/2; %meters (80in)
handles.treadWidth = 1.0160/2; %meters (40in)

% Init Deadbands
handles.posDeadband = 0.05; %meters
handles.velDeadband = 0.05; %meters

% Init Command Limits
handles.absCmdLimit = 1.0; %m/s

% Set shared filenames and sizes
handles.moCapFile = 'rigidBodyData.dat';
handles.moCapSize = 10;
handles.treadFile = 'treadmillData.dat';
handles.treadSize = 5;

% Set initial string values
set(handles.editKp, 'String', handles.Kp);
set(handles.editKd, 'String', handles.Kd);
set(handles.editKi, 'String', handles.Ki);
set(handles.editRefX, 'String', handles.xRef);
set(handles.editRefY, 'String', handles.yRef);
set(handles.editRefZ, 'String', handles.zRef);
set(handles.editCenterX, 'String', handles.centerX);
set(handles.editCenterZ, 'String', handles.centerZ);
set(handles.editPosDeadband, 'String', handles.posDeadband);
set(handles.editVelDeadband, 'String', handles.velDeadband);
set(handles.editAbsCmdLimit, 'String', handles.absCmdLimit);

[handles.vL,handles.vR, handles.incline, handles.aL, handles.aR] = TreadmillState(handles.state, handles.mode, handles.velocity, handles.peakVel, handles.minVel, handles.period, t, handles.optiCmd);

% setup shared data file for mocap and treadmill data
handles.memMoCap = setupDataFile(handles.moCapFile, handles.moCapSize);
handles.memTread = setupDataFile(handles.treadFile, handles.treadSize);

%Create timer
handles.t_update = timer('TimerFcn',{@updateData,hObject}, 'Period',1/100,'ExecutionMode','fixedRate','BusyMode','queue');

% Choose default command line output for TreadmillControl
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Start timer
start(handles.t_update);

%Create signal to send to treadmill
handles.packet = treadmillCommand(handles.vL,handles.vR,handles.aL,handles.aR,handles.incline);

%Send signal to treadmill
fwrite(handles.hTreadmill, handles.packet,'uint8');

% UIWAIT makes TreadmillControl wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

function updateData(~, ~, hObject)
persistent pastDeadband;
persistent errorMem;
persistent dtMem;
persistent lastFrameID;

handles = guidata(hObject);

t = toc;

% Check if data file is ready to be read
if handles.memMoCap.Data(1) == (handles.moCapSize-1)
    
    %Read in data from Optitrack
    [frameRate, frameID, pos] = getMoCapData(handles.memMoCap, handles.moCapSize); 
     % Init frame id
    if isempty(lastFrameID)
        lastFrameID = frameID;
    end
    dt = getDeltaTime(frameRate, frameID, lastFrameID);
    lastFrameID = frameID;
    %Set text displays to view data
    set(handles.textXpos, 'String', pos(1))
    set(handles.textYpos, 'String', pos(2));
    set(handles.textZpos, 'String', pos(3));
    
    % Generate Treadmill Command if Optitrack Mode Enabled
    if handles.mode  == 5
        % Reference Position
        x0 = handles.xRef;
        % Calcuate Position Error
        eX =  x0 - pos(1);
        % Calcuate error deriviative
        eDotX = getErrDot(5,eX,0.0,dt);
        
        if isempty(errorMem)
            errorMem = 0.0;
            dtMem = 0.0;
        end
        
        % Calcuate error integral: absition
        errorMem(end+1) = eX;
        dtMem(end+1) = dtMem(end) + dt;
        eIntX = trapz(dtMem,errorMem);
        
        if ((eIntX > 0.0 && eX < -handles.posDeadband) || (eIntX < 0.0 && eX > handles.posDeadband) || handles.state == 2)
            %reset integrator
            errorMem = 0.0;
            dtMem = 0.0;
            eIntX = 0.0;
        end
        
        % Controller Gains
        Kp = handles.Kp;
        Kd = handles.Kd;
        Ki = handles.Ki;
        
        % check if off treadmill
        if pos(1) >= handles.centerX + handles.treadLength || pos(1) <= handles.centerX - handles.treadLength || ...
                pos(3) >= handles.centerZ + handles.treadWidth || pos(3) <= handles.centerZ - handles.treadWidth
            % robot is out of area of treadmill, turn treadmill off
            handles.optiCmd = 0.0;
            %reset integrator
            errorMem = 0.0;
            dtMem = 0.0;
        else
            
            % Init deadband flag
            if isempty(pastDeadband)
                pastDeadband = false;
            end
            % Position Command Loop with Deadband
            if pastDeadband
                if abs(eX) <= handles.posDeadband
                    posCmd = 0.0;
                    pastDeadband = false;
                else
                    posCmd = Kp * (eX -sign(eX)*handles.posDeadband);
                end
            else
                if abs(eX) >= handles.posDeadband
                    pastDeadband = true;
                    posCmd = Kp * (eX -sign(eX)*handles.posDeadband);
                else
                    posCmd = 0.0;
                end
            end

            if abs(eDotX) <= handles.velDeadband
                velCmd = 0.0;
            else
                velCmd = Kd*(eDotX -sign(eDotX)*handles.velDeadband);
            end
            
            % absement command
            absCmd = Ki * eIntX;
  
            % Check if absCmd is in limits
            if abs(absCmd) > handles.absCmdLimit
                if absCmd >= 0
                    absCmd = handles.absCmdLimit;
                else
                    absCmd = -handles.absCmdLimit;
                end
            end
            
            % Generate Velocity Command for Treadmill
            handles.optiCmd = posCmd + velCmd + absCmd + handles.velocity;
            
            % Send speed data to robot gui
            speedData = [frameRate, frameID, handles.optiCmd, eDotX];
            sendData(speedData, handles.memTread, handles.treadSize);

            % Set text displays to view data
            set(handles.textPosCmd, 'String', posCmd);
            set(handles.textVelCmd, 'String', velCmd);
            set(handles.textAbsCmd, 'String', absCmd);
            set(handles.textTotalCmd, 'String', handles.optiCmd);
        end

    else
        handles.optiCmd = 0.0;
    end
end

% Treadmill State Machine - Checks treadmill control mode and assigns parameters 
[handles.vL,handles.vR, handles.incline, handles.aL, handles.aR] = TreadmillState(handles.state, handles.mode, handles.velocity, handles.peakVel, handles.minVel, handles.period, t, handles.optiCmd);

%Create signal to send to treadmill
handles.packet = treadmillCommand(handles.vL,handles.vR,handles.aL,handles.aR,handles.incline);
handles.t = get(handles.t_update);

%Send signal to treadmill
fwrite(handles.hTreadmill, handles.packet,'uint8')

% Update handles structure
guidata(hObject, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = TreadmillControl_OutputFcn(hObject, eventdata, handles) 
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
handles.state = 1; 
tic;
set(handles.text15, 'String', 'Treadmill On');
% Update handles structure
guidata(hObject, handles);
end

% --- Executes on button press in BTstop.
function BTstop_Callback(hObject, eventdata, handles)
% hObject    handle to BTstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.state = 2; 
set(handles.text15, 'String', 'Treadmill Off');
% Update handles structure
guidata(hObject, handles);
end

function EDvelocity_Callback(hObject, eventdata, handles)
% hObject    handle to EDvelocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EDvelocity as text
%        str2double(get(hObject,'String')) returns contents of EDvelocity as a double
handles.velocity = str2double(get(hObject,'String')); 
guidata(hObject, handles)
return
end

% --- Executes during object creation, after setting all properties.
function EDvelocity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EDvelocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function EDpeakVel_Callback(hObject, eventdata, handles)
% hObject    handle to EDpeakVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EDpeakVel as text
%        str2double(get(hObject,'String')) returns contents of EDpeakVel as a double
handles.peakVel = str2double(get(hObject,'String')); 
guidata(hObject, handles)
return
end

% --- Executes during object creation, after setting all properties.
function EDpeakVel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EDpeakVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function EDminVel_Callback(hObject, eventdata, handles)
% hObject    handle to EDminVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EDminVel as text
%        str2double(get(hObject,'String')) returns contents of EDminVel as a double
handles.minVel = str2double(get(hObject,'String')); 
guidata(hObject, handles)
return
end

% --- Executes during object creation, after setting all properties.
function EDminVel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EDminVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function EDperiod_Callback(hObject, eventdata, handles)
% hObject    handle to EDperiod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EDperiod as text
%        str2double(get(hObject,'String')) returns contents of EDperiod as a double
handles.period = str2double(get(hObject,'String')); 
guidata(hObject, handles)
return
end

% --- Executes during object creation, after setting all properties.
function EDperiod_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EDperiod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on selection change in MENMode.
function MENMode_Callback(hObject, eventdata, handles)
% hObject    handle to MENMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns MENMode contents as cell array
%        contents{get(hObject,'Value')} returns selected item from MENMode
str = get(hObject, 'String');
val = get(hObject, 'Value');
switch str{val}
case 'Constant' 
    disp('Constant')
    handles.mode = 3;
case 'Sine'
    disp('Sine')
    handles.mode = 4;
    tic;
case 'Optitrack'
    disp('Optitrack')
    handles.mode = 5;
end
guidata(hObject, handles)
end         

% --- Executes during object creation, after setting all properties.
function MENMode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MENMode (see GCBO)
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

% close treadmill connection and stop timer
fclose(handles.hTreadmill);
disp('treadmill TCP close');
stop(handles.t_update);

% Update handles structure
guidata(hObject, handles);
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% delete timer and close all open files
delete(handles.t_update);
fclose('all');

% Update handles structure
guidata(hObject, handles);

% Hint: delete(hObject) closes the figure
delete(hObject);
end

function editKp_Callback(hObject, eventdata, handles)
% hObject    handle to editKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKp as text
%        str2double(get(hObject,'String')) returns contents of editKp as a double
handles.Kp = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editKd_Callback(hObject, eventdata, handles)
% hObject    handle to editKd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKd as text
%        str2double(get(hObject,'String')) returns contents of editKd as a double
handles.Kd = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editRefX_Callback(hObject, eventdata, handles)
% hObject    handle to editRefX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editRefX as text
%        str2double(get(hObject,'String')) returns contents of editRefX as a double
handles.xRef = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editRefX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editRefX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editRefY_Callback(hObject, eventdata, handles)
% hObject    handle to editRefY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editRefY as text
%        str2double(get(hObject,'String')) returns contents of editRefY as a double
handles.yRef = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editRefY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editRefY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editRefZ_Callback(hObject, eventdata, handles)
% hObject    handle to editRefZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editRefZ as text
%        str2double(get(hObject,'String')) returns contents of editRefZ as a double
handles.xRef = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editRefZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editRefZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editKi_Callback(hObject, eventdata, handles)
% hObject    handle to editKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editKi as text
%        str2double(get(hObject,'String')) returns contents of editKi as a double
handles.Ki = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editKi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editPosDeadband_Callback(hObject, eventdata, handles)
% hObject    handle to editPosDeadband (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editPosDeadband as text
%        str2double(get(hObject,'String')) returns contents of editPosDeadband as a double
handles.posDeadband = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editPosDeadband_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editPosDeadband (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editVelDeadband_Callback(hObject, eventdata, handles)
% hObject    handle to editVelDeadband (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVelDeadband as text
%        str2double(get(hObject,'String')) returns contents of editVelDeadband as a double
handles.velDeadband = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editVelDeadband_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVelDeadband (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editCenterX_Callback(hObject, eventdata, handles)
% hObject    handle to editCenterX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editCenterX as text
%        str2double(get(hObject,'String')) returns contents of editCenterX as a double
handles.centerX = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editCenterX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editCenterX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editCenterZ_Callback(hObject, eventdata, handles)
% hObject    handle to editCenterZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editCenterZ as text
%        str2double(get(hObject,'String')) returns contents of editCenterZ as a double
handles.centerZ = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editCenterZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editCenterZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function editAbsCmdLimit_Callback(hObject, eventdata, handles)
% hObject    handle to editAbsCmdLimit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAbsCmdLimit as text
%        str2double(get(hObject,'String')) returns contents of editAbsCmdLimit as a double
handles.absCmdLimit = str2double(get(hObject,'String')); 
guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function editAbsCmdLimit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAbsCmdLimit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
