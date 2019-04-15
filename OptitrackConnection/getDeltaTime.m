function dt = getDeltaTime(OptiFrameRate, frameID, lastFrameID)

frameDiff = (frameID - lastFrameID);
% calcuate time since last frame (secs)
dt = cast(frameDiff,'double') / OptiFrameRate;
