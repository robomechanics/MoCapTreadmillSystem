function [vL, vR, incline, aL, aR] = TreadmillState(state, mode, velocity, peak, minVel, period, t, optiCmd)
% Treadmill State Machine

%Defining states and modes
treadOn = 1;    % Treadmill On
treadOff = 2;   % Treadmill Off
velConst = 3;   % Constant Velocity Manual Control
velSine = 4;    % Sine Wave Manual Control
optitrack = 5;  % Optitrack Automated Control

if state == treadOff 
     vL = 0;
     vR = 0;
     aL = .7;
     aR = .7;
     incline = 0;
elseif  state == treadOn
    % Constant Velocity Manual Control
    if  mode == velConst 
         vL = velocity;
         vR = velocity;
         aL = .7;
         aR = .7;
         incline = 0;
    %Sine Wave Manual Control
    elseif  mode == velSine 
         vL = 0.5*(peak-minVel)*sin(2*pi*t/period) + 0.5*(peak-minVel)+minVel;
         vR = 0.5*(peak-minVel)*sin(2*pi*t/period) + 0.5*(peak-minVel)+minVel;
         aL = 1;
         aR = 1;
         %aL = (peak-minVel)*pi/period*cos(2*pi*t/period + pi);
         %aR = (peak-minVel)*pi/period*cos(2*pi*t/period + pi);
         incline = 0;
    % Optitrack Automated Control
    elseif mode == optitrack
         vL = optiCmd;
         vR = optiCmd;
         aL = 5.0;
         aR = 5.0;
         incline = 0;
    else
        state = treadOff;
    end
else 
     vL = 0;
     vR = 0;
     aL = .7;
     aR = .7;
     incline = 0;
end


