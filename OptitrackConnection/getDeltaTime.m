function dt = getDeltaTime(OptiFrameRate, frameID)

persistent lastFrameID;

% Init frame id
if isempty(lastFrameID)
    lastFrameID = frameID;
end
    
frameDiff = (frameID - lastFrameID);
% calcuate time since last frame (secs)
dt = cast(frameDiff,'double') / OptiFrameRate;
lastFrameID = frameID;