function [msg] = getData(memLoc,size)

msg = (memLoc.Data(2:size));
%disp('Received message from SEND:')
%disp(msg)
memLoc.Data(1) = 0;
