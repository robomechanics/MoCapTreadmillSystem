function sendData_sync(tcpObj, cmdPacket)

flushinput(tcpObj);
fprintf(tcpObj,'%c','RM');
for ii = 1:length(cmdPacket)
    fprintf(tcpObj, '%d', cmdPacket(ii));
    fprintf(tcpObj,'%c',',');
end
