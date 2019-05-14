
function cost = costFunction_Minitaur_EnergyPerDistance(x, hObject)

handles = guidata(hObject);

optGaitParams = [x(1) x(2) 0.009 0.0 0.0 0.0 0.0];
disp('New Gait Parameters')
disp(optGaitParams)
%optGaitParams = [dutyFactor, period, thetaDown, thetaSlow, Kp, Kd];
%gait used for recentering
%regGaitParams = [height extMin];
regGaitParams = [2 0.4 0.009 0.0 0.0 0.0 0.0];
coolDownGaitParams = [0.45 0.4 0.009 0.0 0.0 0.0 0.0];
optState = 'restart';
maxTempVal = 70.0;
restartTempVal = 45.0;

%init tcp data
voltage = 0.0;
current = 0.0;
pTime = 0.0;
motorTempFlag = 0;
maxMotorTemp = 0.0;

%check if params are in bounds
if x(1) < handles.PLowBound(1) || x(1) > handles.PUpBound(1) || x(2) < handles.PLowBound(2) || x(2) > handles.PUpBound(2)
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
        if handles.tcpObj.BytesAvailable >= 20
            tcpData = fread(handles.tcpObj,5,'float32');
            while handles.tcpObj.BytesAvailable > 100
                tcpData = fread(handles.tcpObj,5,'float32');
                disp('num Bytes')
                disp(handles.tcpObj.BytesAvailable)
            end
            voltage = tcpData(1);
            current = tcpData(2);
            pTime = tcpData(3);
            motorTempFlag = tcpData(4);
            maxMotorTemp = tcpData(5);
        else
            disp('not enough bytes')
            disp(handles.tcpObj.BytesAvailable)
        end
    catch
        disp('failed to read')
        disp(handles.tcpObj.BytesAvailable)
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
            pTimeLast = pTime;
            handles.restartOpt = false;
            optState = 'recenter';
               
        case 'pause' % pause trial
            %stop robot and set regular params
            cmdPacket = [0.0 0.0 regGaitParams ];
            sendData_sync(handles.tcpObj, cmdPacket);
            %fwrite(handles.tcpObj, cmdPacket,'double');
            
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
            %fwrite(handles.tcpObj, cmdPacket,'double');

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
            pdt = pTime - pTimeLast;
            currEnergy = voltage*current*pdt;
            totalEnergy = totalEnergy + currEnergy;

            if handles.recordTrial
                [rows,cols] = size(handles.trialData, 'dt');
                handles.trialData.dt(rows+1,:) = dt;
                handles.trialData.dist(rows+1,:) = dist;
                handles.trialData.energy(rows+1,:) = currEnergy;
                handles.trialData.voltage(rows+1,:) = voltage;
                handles.trialData.current(rows+1,:) = current;
                handles.trialData.totalEnergy(rows+1,:) = totalEnergy;
                handles.trialData.pdt(rows+1,:) = pdt;
            end

            % check if trial is complete & update robot
            if totalTime >= handles.trialLength
                cost = totalEnergy/totalDist;
                [rows,cols] = size(handles.optData, 'cost');
                handles.optData.cost(rows+1,:) = cost;
                handles.optData.gait(rows+1,:) = optGaitParams;
                disp('Cost')
                disp(cost)
                trialActive = false;
                
                %stop optimization gait
                cmdPacket = [0.0 0.0 optGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
                %fwrite(handles.tcpObj, cmdPacket,'double');
            else
                %command optimization gait
                cmdPacket = [handles.fwdVel cmdData.cmd optGaitParams];
                sendData_sync(handles.tcpObj, cmdPacket);
                %fwrite(handles.tcpObj, cmdPacket,'double');
            end
        case 'fail' %Case out of bounds, return high cost
            
            %stop optimization gait
            cmdPacket = [0.0 0.0 regGaitParams];
            sendData_sync(handles.tcpObj, cmdPacket);
            %fwrite(handles.tcpObj, cmdPacket,'double');
            
            trialActive = false;
            
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
            
            cost = 100*diff1+100*diff2;
            
        case 'coolDown' %motor temps to high, let them cool down    
            disp('cooldown')
            cmdPacket = [0.0 0.0 coolDownGaitParams ];
            sendData_sync(handles.tcpObj, cmdPacket);
            %fwrite(handles.tcpObj, cmdPacket,'double');
            disp(maxMotorTemp)
            if maxMotorTemp < restartTempVal 
                optState = 'restart';
            end

    end
    
    pTimeLast = pTime;
    
    % Update handles structure
    guidata(hObject, handles);
end