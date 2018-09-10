function [eDotPos,eDotEuler] = getErrDot(devSize, ePos, eEuler, dt)

persistent dtArray;
persistent eEulerArray;
persistent ePosArray;

% Save points to calculate derivative
if isempty(dtArray)
    eEulerArray(1:devSize) = eEuler;
    ePosArray(1:devSize) = ePos;
    dtArray(1:devSize) = dt;
else
    % save attitude error values
    eEulerArray = eEulerArray(2:end);
    eEulerArray(end+1) = eEuler;
    % save position error values
    ePosArray = ePosArray(2:end);
    ePosArray(end+1) = ePos;
    % save dt values
    dtArray = dtArray(2:end);
    dtArray(end+1) = dt;
end

% Calcuate attitude and position error deriviatives
eEulerDiff = diff(eEulerArray);
eDotEuler = eEulerDiff/dtArray(1:end-1);
ePosDiff = diff(ePosArray);
eDotPos = ePosDiff/dtArray(1:end-1);
