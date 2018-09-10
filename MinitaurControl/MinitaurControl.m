function varargout = MinitaurControl(varargin)
% MINITAURCONTROL MATLAB code for MinitaurControl.fig
%      MINITAURCONTROL, by itself, creates a new MINITAURCONTROL or raises the existing
%      singleton*.
%
%      H = MINITAURCONTROL returns the handle to a new MINITAURCONTROL or the handle to
%      the existing singleton*.
%
%      MINITAURCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MINITAURCONTROL.M with the given input arguments.
%
%      MINITAURCONTROL('Property','Value',...) creates a new MINITAURCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MinitaurControl_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MinitaurControl_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MinitaurControl

% Last Modified by GUIDE v2.5 14-Jun-2018 00:48:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MinitaurControl_OpeningFcn, ...
                   'gui_OutputFcn',  @MinitaurControl_OutputFcn, ...
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


% --- Executes just before MinitaurControl is made visible.
function MinitaurControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MinitaurControl (see VARARGIN)

% USER INPUTS
%serverAddress = 'localhost';
serverAddress = '128.237.232.147';
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

%Init Values
handles.fwdVel = 0.0;
handles.angVel = 0.0;
handles.stHgt = 0.0;
handles.P1 = 0.0;
handles.P2 = 0.0;
handles.P3 = 0.0;
handles.P4 = 0.0;
handles.P5 = 0.0;
handles.P6 = 0.0;
handles.zRef = 0.475;
handles.yawRef = 4.0;

% Set initial gain values
handles.KpZpos = 2.0;
handles.KdZpos = 0.0;
handles.KpYaw = 0.1;
handles.KdYaw = 0.0;

% Update gui displays
set(handles.editZRef, 'String', handles.zRef);
set(handles.editYawRef, 'String',  handles.yawRef);
set(handles.editKpZpos, 'String', handles.KpZpos);
set(handles.editKdZpos, 'String',  handles.KdZpos);
set(handles.editKpYaw, 'String',  handles.KpYaw);
set(handles.editKdYaw, 'String',  handles.KdYaw);

% Set Manual Control Limits
handles.fwdVelLimit = 1.0;
handles.angVelLimit = 0.5;

% TODO: move m and vSize into handles to reduce variables being passed into
% timer
% setup data file and memory mapping
[handles.m,handles.vSize] = setupDataFile('rigidBodyData.dat');

%Create timer
handles.t_update = timer('TimerFcn',{@updateData,hObject}, 'Period',1/100,'ExecutionMode','fixedRate','BusyMode','queue');

% Choose default command line output for MinitaurControl
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Start timer
start(handles.t_update);

% UIWAIT makes MinitaurControl wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

function updateData(~, ~, hObject)

persistent angVelCmd;
persistent dtArray;
persistent eYawArray;
persistent eZArray;

%get handles struct
handles = guidata(hObject);

% Check if data file is ready to be read
if handles.m.Data(1) == (handles.vSize-1)
    %Read in Data from Optitrack
    msg = getData(handles.m,handles.vSize);
    % delta time
    dt = msg(1);
    % postions X, Y and Z
    pos = msg(2:4);
    % orientation quaternion
    quat = msg(5:8);
    
    % convert to euler angles
    q = quaternion( quat(1), quat(2), quat(3), quat(4) );
    qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
    q = mtimes(q, qRot);
    eulerRad = EulerAngles(q,'zyx');
    eulerDeg(1) = -eulerRad(1) * 180.0 / pi;   % must invert due to 180 flip above
    eulerDeg(2) = eulerRad(2) * 180.0 / pi;
    eulerDeg(3) = -eulerRad(3) * 180.0 / pi;   % must invert due to 180 flip above

    %Set text displays to view data
    set(handles.textXpos, 'String', pos(1));
    set(handles.textYpos, 'String', pos(2));
    set(handles.textZpos, 'String', pos(3));
    set(handles.textXang, 'String', eulerDeg(1));
    set(handles.textYang, 'String', eulerDeg(2));
    set(handles.textZang, 'String', eulerDeg(3));
    
    %check if in Optitrack Control Mode
    if handles.mode == 2
        % Yaw Angular Velocity Controller
        % Yaw and Z pos Error
        eYaw = handles.yawRef - eulerDeg(2);
        eZ = handles.zRef - pos(3);

        % Number of points taken for dervitaive
        devSize = 5;

        % Save points to calculate derivative
        if isempty(dtArray)
            eYawArray(1:devSize) = eYaw;
            eZArray(1:devSize) = eZ;
            dtArray(1:devSize) = dt;
        else
            % save Yaw error values
            eYawArray = eYawArray(2:end);
            eYawArray(end+1) = eYaw;
            % save Z error values
            eZArray = eZArray(2:end);
            eZArray(end+1) = eZ;
            % save dt values
            dtArray = dtArray(2:end);
            dtArray(end+1) = dt;
        end
        
        % Calcuate Yaw and Z pos error deriviatives
        eYawDiff = diff(eYawArray);
        eDotYaw = eYawDiff/dtArray(1:end-1);
        eZDiff = diff(eZArray);
        eDotZ = eZDiff/dtArray(1:end-1);

        % Attitude and Position Commands
        attCmd = -handles.KpYaw * eYaw + handles.KdYaw * eDotYaw;
        posCmd = handles.KpZpos * eZ + handles.KdZpos * eDotZ;
        % Total Yaw Angular Velocity Command
        angVelCmd = attCmd + posCmd;

        %for gain tuning
        set(handles.text29, 'String', attCmd);
        set(handles.text30, 'String', posCmd);
        set(handles.text31, 'String', angVelCmd);


    end

else
    %initialize yaw command
    if isempty(angVelCmd)
        angVelCmd = 0.0;
    end
end

%Check Control Mode
if handles.mode == 1
    %Manual Control Mode
    angVel = handles.angVel;
elseif handles.mode == 2
    %Optitrack Control Mode
    angVel = angVelCmd; 
end

%Check if robot control is on
if handles.robotOn
    %assemble python data packet here
    %TODO: add parameters to python packet
    % posError, mode, empty
    pyPacket = [angVel 1.0 0.0];
else
    %set data packet cmds to zero
    pyPacket = [0.0, 0.0, 0.0];
end

%Send signal to robot
fwrite(handles.tcpObj, pyPacket,'double')

% Update handles structure
guidata(hObject, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = MinitaurControl_OutputFcn(hObject, eventdata, handles) 
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

%TODO: Move this to a better place to control values
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

%TODO: Move this to a better place to control values
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
switch str{val};
case 'Manual Control' 
    handles.mode = 1;
case 'Optitrack Angular Control'
    handles.mode = 2;
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
