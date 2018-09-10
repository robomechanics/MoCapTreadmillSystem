function [frameRate, frameID, pos, eulerDeg] = getMoCapData(m, vSize)
%Read in Data from Optitrack
msg = getData(m,vSize);
% frame rate of optitrack data
frameRate = msg(1);
% current frame number
frameID = msg(2);
% postions X, Y and Z
pos = msg(3:5);
% orientation quaternion
quat = msg(6:9);

% convert to euler angles
q = quaternion( quat(1), quat(2), quat(3), quat(4) );
qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
q = mtimes(q, qRot);
eulerRad = EulerAngles(q,'zyx');
eulerDeg(1) = -eulerRad(1) * 180.0 / pi;   % must invert due to 180 flip above
eulerDeg(2) = eulerRad(2) * 180.0 / pi;
eulerDeg(3) = -eulerRad(3) * 180.0 / pi;   % must invert due to 180 flip above