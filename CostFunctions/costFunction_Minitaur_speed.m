
function cost = costFunction_Minitaur_speed(x, hObject)

handles = guidata(hObject);

optGaitParams = [x(1) x(2) x(3) x(4) x(5) 0.0 0.0];
disp('New Gait Parameters')
disp(optGaitParams)
%gait used for recentering
%Clark Trot Gait
%regGaitParams = [Frequency DutyFactor StrokeLength ApproachAngle ExtensionKp ExtensionKd Unused];
regGaitParams = [2.0 50.0 0.12 40.0 180.0 1.8 0.0];
coolDownGaitParams = [2.0 50.0 0.12 40.0 180.0 1.8 0.0]; %will just switch to SIT behavior
optState = 'restart';
maxTempVal = 85.0;
restartTempVal = 50.0;

%init tcp data
motorTempFlag = 0;
maxMotorTemp = 0.0;
totEng = 0.0;
startEnergy = [];

%check if params are in bounds
if x(1) < handles.PLowBound(1) || x(1) > handles.PUpBound(1) ...
        || x(2) < handles.PLowBound(2) || x(2) > handles.PUpBound(2) ...
        || x(3) < handles.PLowBound(3) || x(3) > handles.PUpBound(3) ...
        || x(4) < handles.PLowBound(4) || x(4) > handles.PUpBound(4) ...
        || x(5) < handles.PLowBound(5) || x(5) > handles.PUpBound(5)
    optState = 'fail';
else
    optState = 'restart';
end
trialActive = true;

while(trialActive)   
    % Note: sleep() accepts [mSecs] duration, but does not process any events.
    % pause() processes events, but resolution on windows can be at worst 15 msecs
    %java.lang.Thread.sleep(10);  
    pause(1/25); % max rate pause can do

    %get updated handles struct
    handles = guidata(hObject);
    
    try
        %try to get Minitaur Data
        tcpData = recvData_sync(handles.tcpObj);
        if ~isempty(tcpData)
            %set values
            totEng = tcpData(1);
            motorTempFlag = tcpData(2);
            maxMotorTemp = tcpData(3);
        end
        
    catch
        continue;
    end
    
    if motorTempFlag == 1 || maxMotorTemp > maxTempVal || strcmp(optState,'coolDown')
       optState = 'coolDown';
    elseif handles.pauseOpt
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
            totalEnergy = 0.0;
            totEng = 0.0;
            startEnergy = [];
            handles.restartOpt = false;
            optState = 'recenter';
            handles.trialData = clearTrialRecord(handles.trialData);
               
        case 'pause' % pause trial
            %stop robot and set regular params
            cmdPacket = [0.0 0.0 regGaitParams];
            sendData_sync(handles.tcpObj, cmdPacket);
            
            if ~handles.pauseOpt 
                optState = lastState;
            end
            
        case 'recenter' % Recenter robot for trial
            % Calculate yaw command and check if centered on treadmill
            [cmdData, handles] = calcYawCmd(handles);

            %check if the robot is in position
            if ~isnan(cmdData.attError) && abs(cmdData.attError) <=  handles.yawCenterLimit && abs(cmdData.posError) <= handles.zCenterLimit
                %stop robot and wait for trial to begin
                cmdPacket = [0.0 0.0 regGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
                %fwrite(handles.tcpObj, cmdPacket,'double');
                optState = 'trailToSS';
                disp('trailToSS')
            else
                %recenter robot with regular params
                cmdPacket = [handles.fwdVel cmdData.cmd regGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
                %fwrite(handles.tcpObj, cmdPacket,'double');
            end

        case 'trailToSS' % Get to steady state operation of gait
            % Calculate yaw command and check if centered on treadmill
            [cmdData, handles] = calcYawCmd(handles);
            
            %command optimization gait
            cmdPacket = [handles.fwdVel cmdData.cmd optGaitParams];
            sendData_sync(handles.tcpObj, cmdPacket);

            % Get treadmill data
            [dist, dt] = getTreadData(handles.memTread, handles.treadSize);
            
            % Calculate total time
            totalTime = totalTime + dt;
                
            %start recording once at steady state
            if totalTime >= handles.timeToSS
                totalTime = 0.0;
                totalDist = 0.0;
                totalEnergy = 0.0;
                optState = 'trialRecord';
                disp('trialRecord')
            end

        case 'trialRecord' % Record trial and calculate cost
%             if isempty(startEnergy)
%                 startEnergy = totEng;
%             end
            
            % Calculate yaw command and check if centered on treadmill
            [cmdData, handles] = calcYawCmd(handles);
            
            % Get treadmill data
            [dist, dt] = getTreadData(handles.memTread, handles.treadSize);
            
            if handles.reverseDirection
                dist = -dist;
            end
                        
            % Calculate total time and distance
            totalTime = totalTime + dt;
            totalDist = totalDist + dist;
%             if totEng < startEnergy
%                 %check if float wrapped around (assuming this will only
%                 %happen once per trial max)
%                 totalEnergy = totEng +(realmax('single') - startEnergy);
%                 disp('wrap happened')
%             else
%                 totalEnergy = totEng - startEnergy;
%             end
                
            %totalEnergy = totalEnergy + currEnergy;

            if handles.recordTrial
                [rows,cols] = size(handles.trialData, 'dt');
                handles.trialData.dt(rows+1,:) = dt;
                handles.trialData.dist(rows+1,:) = dist;
%                 handles.trialData.energy(rows+1,:) = currEnergy;
%                 handles.trialData.voltage(rows+1,:) = voltage;
%                 handles.trialData.current(rows+1,:) = current;
                handles.trialData.totalEnergy(rows+1,:) = totalEnergy;
                %handles.trialData.pdt(rows+1,:) = pdt;
            end

            % check if trial is complete & update robot
            if totalTime >= handles.trialLength
                %stop optimization gait
                cmdPacket = [0.0 0.0 optGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
                
                %cost = totalEnergy/totalDist;
                cost = totalTime/totalDist;
                [rows,cols] = size(handles.optData, 'cost');
                handles.optData.cost(rows+1,:) = cost;
                handles.optData.gait(rows+1,:) = optGaitParams;
                disp('Cost')
                disp(cost)
                trialActive = false;
                startEnergy = [];
                
                if cost < handles.recordThresh && handles.recordTrial
                    rec = initTrialRecord(['trialData_' num2str(rows+1)]);
                    [rows,cols] = size(handles.trialData, 'dt');
                    rec.dt = handles.trialData.dt;
                    rec.dist = handles.trialData.dist;
                    %rec.energy= handles.trialData.energy;
                    %rec.voltage = handles.trialData.voltage;
                    %rec.current = handles.trialData.current;
                    rec.totalEnergy = handles.trialData.totalEnergy;
                    %rec.pdt = handles.trialData.pdt;
                end
                %stop optimization gait
                cmdPacket = [0.0 0.0 regGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
            else
                %command optimization gait
                cmdPacket = [handles.fwdVel cmdData.cmd optGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
            end
        case 'fail' %Case out of bounds, return high cost
            
            %stop optimization gait
            cmdPacket = [0.0 0.0 regGaitParams];
            sendData_sync(handles.tcpObj, cmdPacket);
            
            if x(1) < handles.PLowBound(1)
                diff1 = handles.PLowBound(1) - x(1); 
            elseif x(1) > handles.PUpBound(1)
                diff1 = x(1) - handles.PUpBound(1);
            else
                diff1 = 0;
            end
            
            if x(2) < handles.PLowBound(2)
                diff2 = handles.PLowBound(2) - x(2);
            elseif x(2) > handles.PUpBound(2)
                diff2 = x(2) - handles.PUpBound(2);
            else
                diff2 = 0;
            end
            
            cost = 10e4*diff1+10e4*diff2+10000;
            [rows,cols] = size(handles.optData, 'cost');
            handles.optData.cost(rows+1,:) = cost;
            handles.optData.gait(rows+1,:) = optGaitParams;
            disp('Cost')
            disp(cost)
            
            trialActive = false;
            
        case 'coolDown' %motor temps to high, let them cool down    
            disp('cooldown')
            cmdPacket = [0.0 0.0 coolDownGaitParams ];
            sendData_sync(handles.tcpObj, cmdPacket);
            %TODO: Move this to GUI Display
            disp(maxMotorTemp)
            if maxMotorTemp < restartTempVal 
                optState = 'restart';
            end

    end

    % Update handles structure
    guidata(hObject, handles);
end