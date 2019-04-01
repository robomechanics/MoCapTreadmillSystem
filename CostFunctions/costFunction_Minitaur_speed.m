
function cost = costFunction_Minitaur_speed(x, hObject)

optGaitParams = [x(1) x(2) 0.0 0.0 0.0 0.0 0.0];
disp('New Gait Parameters')
disp(optGaitParams)
%optGaitParams = [dutyFactor, period, thetaDown, thetaSlow, Kp, Kd];
%gait used for recentering
%regGaitParams = [height extMin];
regGaitParams = [1.5 0.3 0.0 0.0 0.0 0.0 0.0];
optState = 'restart';
trialActive = true;

while(trialActive)   
    % Note: sleep() accepts [mSecs] duration, but does not process any events.
    % pause() processes events, but resolution on windows can be at worst 15 msecs
    %java.lang.Thread.sleep(10);  
    pause(1/25); % max rate pause can do

    %get updated handles struct
    handles = guidata(hObject);

    if handles.pauseOpt
        if ~strcmp(optState,'pause')
            lastState = optState;
        end
        optState = 'pause';
    elseif handles.restartOpt
        optState = 'restart';
        handles.restratOpt = false;
    end

    switch optState
        case 'restart' % restart trial
            % init values
            totalTime = 0.0;
            totalDist = 0.0;
            handles.restartOpt = false;
            optState = 'recenter';
               
        case 'pause' % pause trial
            %stop robot and set regular params
            cmdPacket = [0.0 0.0 regGaitParams ];
            fwrite(handles.tcpObj, cmdPacket,'double');
            
            if ~handles.pauseOpt
                optState = lastState;
            end
            
        case 'recenter' % Recenter robot for trial
            disp('recenter')
            % Calculate yaw command and check if centered on treadmill
            [cmdData, handles] = calcYawCmd(handles);

            %check if the robot is in position
            if ~isnan(cmdData.attError) && abs(cmdData.attError) <=  handles.yawCenterLimit && abs(cmdData.posError) <= handles.zCenterLimit
                %stop robot and wait for trial to begin
                cmdPacket = [0.0 0.0 regGaitParams];
                fwrite(handles.tcpObj, cmdPacket,'double');
                optState = 'trailToSS';
            else
                %recenter robot with regular params
                cmdPacket = [handles.fwdVel cmdData.cmd regGaitParams];
                fwrite(handles.tcpObj, cmdPacket,'double');
            end

        case 'trailToSS' % Get to steady state operation of gait
            disp('trailToSS')
            % Calculate yaw command and check if centered on treadmill
            [cmdData, handles] = calcYawCmd(handles);
            
            %command optimization gait
            cmdPacket = [handles.fwdVel cmdData.cmd optGaitParams];
            fwrite(handles.tcpObj, cmdPacket,'double');

            % Get treadmill data
            [dist, dt] = getTreadData(handles.memTread, handles.treadSize);
            
            % Calculate total time
            totalTime = totalTime + dt;
                
            %start recording once at steady state
            if totalTime >= handles.timeToSS
                totalTime = 0.0;
                totalDist = 0.0;
                optState = 'trialRecord';
            end

        case 'trialRecord' % Record trial and calculate cost
            % Calculate yaw command and check if centered on treadmill
            disp('trialRecord')
            [cmdData, handles] = calcYawCmd(handles);
            
            % Get treadmill data
            [dist, dt] = getTreadData(handles.memTread, handles.treadSize);
            
            % Get power data
            powerData = fread(handles.tcpObj,2,'float32')
            
            % Calculate total time and distance
            totalTime = totalTime + dt;
            totalDist = totalDist + dist;

            % check if trial is complete & update robot
            if totalDist >= handles.trialLength
                cost = 1/(totalDist/totalTime);
                [rows,cols] = size(handles.optData, 'cost');
                handles.optData.cost(rows+1,:) = cost;
                handles.optData.gait(rows+1,:) = optGaitParams;
                disp('Cost')
                disp(cost)
                trialActive = false;
                
                %stop optimization gait
                cmdPacket = [0.0 0.0 optGaitParams];
                fwrite(handles.tcpObj, cmdPacket,'double');
            else
                %command optimization gait
                cmdPacket = [handles.fwdVel cmdData.cmd optGaitParams];
                fwrite(handles.tcpObj, cmdPacket,'double');
            end

    end
    
    % Update handles structure
    guidata(hObject, handles);
end