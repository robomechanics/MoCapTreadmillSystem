function sendData_sync(tcpObj, cmdPacket)

%flushinput(tcpObj);
fprintf(tcpObj,'%c','RM');
%fprintf(tcpObj, '%f', cmdPacket);
cmdPacket = cast(cmdPacket,'single');
fwrite(tcpObj, cmdPacket,'float');
%checksum = cast(2 + length(cmdPacket)*4 + 2,'uint16');
checksum = 0;
for ii = 1:length(cmdPacket)
    temp = de2bi(cmdPacket(ii))
    checksum = checksum + de2bi(cmdPacket(ii))
end
checksum %= cast(checksum,'uint16')
%fwrite(tcpObj, checksum,'uint16');
% for ii = 1:length(cmdPacket)
%     fprintf(tcpObj, '%d', cmdPacket(ii));
%     fprintf(tcpObj,'%c',',');
% end
